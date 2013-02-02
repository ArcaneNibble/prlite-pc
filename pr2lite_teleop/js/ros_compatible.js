/*
 * Provides the same public API as ros.js for rosbridge v1.0
 * 
 * Not the greatest of APIs, it's a better idea to use the new ros.js
 * 
 * Provided for backwards-compatibility
 * 
 */

var ros = ros || {};

var Bridge = function(url) {
    // Initialize internal variables
    this.service_handlers = {};
    this.service_seed = 0;
    this.subscription_handlers = {};
    
    // Ensure that JSON and WebSocket are available.  Thrown an exception if not
    if (!WebSocket && MozWebSocket) {
        WebSocket = MozWebSocket;
    }
    if (!WebSocket) {
        throw "Browser does not support WebSockets";
    }
    if (!JSON) {
        throw "Browser does not support JSON";
    }
    
    // Create the connection
    this.socket = new WebSocket(url);
    
    var self = this
    this.socket.onmessage = function() {
        self.receiveMessage.apply(self, arguments);
        self.onMessage.apply(self, arguments);
    }
    this.socket.onerror = function() {
        self.onError.apply(self, arguments);
    }
    this.socket.onopen = function() {
        self.onOpen.apply(self, arguments);
    }
    this.socket.onclose = function() {
        self.onClose.apply(self, arguments);
    }
}

Bridge.prototype.send = function(op, id, msg) {
    msg.op = op;
    if (id != null) {
        msg.id = id;
    }
    console.log("Sending", JSON.stringify(msg));
    this.socket.send(JSON.stringify(msg));
}

Bridge.prototype.advertise = function(topic, type, /*optional*/ id) {
    this.send("advertise", id, {topic: topic, type: type});
}

Bridge.prototype.unadvertise = function(topic, /*optional*/ id) {
    this.send("unadvertise", id, {topic: topic});
}

Bridge.prototype.publish = function(topic, msg, /*optional*/ id) {
    this.send("publish", id, {topic: topic, msg: msg});
}

Bridge.prototype.subscribe = function(callback, topic, /*optional*/ type, /*optional*/ throttle_rate, 
      /*optional*/ queue_length, /*optional*/ fragment_size, /*optional*/ compression, /*optional*/ id) {
    // Construct the message
    msg = {topic: topic};
    if (type != null) msg.type = type;
    if (throttle_rate != null && throttle_rate != -1) msg.throttle_rate = throttle_rate;
    if (queue_length != null) msg.queue_length = queue_length;
    if (fragment_size != null) msg.fragment_size = fragment_size;
    if (compression != null) msg.compression = compression;
    
    // Send the message
    this.send("subscribe", id, msg);
    
    // Save the callback
    if (this.subscription_handlers[topic] == null) {
        this.subscription_handlers[topic] = {};
    }
    if (this.subscription_handlers[topic][id] == null) {
        this.subscription_handlers[topic][id] = [];
    }
    this.subscription_handlers[topic][id].push(callback);
}

Bridge.prototype.unsubscribe = function(topic, /*optional*/ id) {
    // Send the message
    this.send("unsubscribe", id, {topic: topic});
    
    // Delete callbacks
    if (this.subscription_handlers[topic] && this.subscription_handlers[topic][id]) {
        delete this.subscription_handlers[topic][id];
    }
    if (id==null || Object.keys(this.subscription_handlers[topic]).length == 0) {
        delete this.subscription_handlers[topic];
    }
}

Bridge.prototype.callService = function(callback, service, /*optional*/ args,
        /*optional*/ fragment_size, /*optional*/ compression, /*optional*/ id) {
    // Construct the message
    msg = {service: service};
    if (args != null) msg.args = args;
    if (fragment_size != null) msg.fragment_size = fragment_size;
    if (compression != null) msg.compression = compression;
    
    // Generate an ID for service calls
    if (id == null) {
        id = this.service_seed;
        this.service_seed++;
    }
    
    // Send the message
    this.send("call_service", id, msg);
    
    // Save the callback
    if (this.service_handlers[service] == null) {
        this.service_handlers[service] = {};
    }
    this.service_handlers[service][id] = callback;
}

Bridge.prototype.receiveMessage = function(event) {
    console.log("Received incoming", event.data);
    msg = JSON.parse(event.data);
    
    switch(msg.op) {
        case "publish": this.onPublish(msg); break;
        case "service_response": this.onServiceResponse(msg); break;
    }
}

Bridge.prototype.onOpen = function(event) {}
Bridge.prototype.onClose = function(event) {}
Bridge.prototype.onError = function(event) {}
Bridge.prototype.onMessage = function(event) {}

Bridge.prototype.onPublish = function(message) {
    // Extract message details
    topic = message.topic;
    msg = message.msg;
    
    // Copy the callbacks - in case the callback modifies the subscription
    var callbacks = [];
    for (var id in this.subscription_handlers[topic]) {
        callbacks = callbacks.concat(this.subscription_handlers[topic][id]);
    }
    
    // Call all the callbacks
    for (var i = 0; i < callbacks.length; i++) {
        try {
            callbacks[i](msg);
        } catch (err) {
            // Best we can do is print the error
            console.error(err);
        }
    }
}

Bridge.prototype.onServiceResponse = function(response) {
    // Extract message details
    service = response.service;
    values = response.values;
    id = response.id;
    
    // Call the callback and remove it
    if (this.service_handlers[service] && this.service_handlers[service][id]) {
        callback = this.service_handlers[service][id];
        delete this.service_handlers[service][id];
        if (Object.keys(this.service_handlers[service]).length == 0) {
            delete this.service_handlers[service];
        }
        callback(values);
    }
}

var Connection = function(url) {
    this.bridge = new Bridge(url);
    this.socket = this.bridge.socket;
    this.advertised = {};
}

Connection.prototype.callService = function(service, payload, callback) {
    service = service.replace("/rosjs/", "/rosbridge/");
    if (typeof payload == "string") {
        payload = JSON.parse(payload);
    }
    switch (service) {
        case "/rosbridge/subscribe": this._subscribe(service, payload, callback); break;
        case "/rosbridge/unsubscribe": this._unsubscribe(service, payload, callback); break;
        default: this.bridge.callService(callback, service, payload);
    }
}

Connection.prototype._subscribe = function(service, payload, callback) {
    var topic = payload[0];
    var throttle_rate = payload[1];
    var type = payload[2];
    this.bridge.subscribe(callback, topic, type, throttle_rate);
}

Connection.prototype._unsubscribe = function(service, payload, callback) {
    var topic = payload[0];
    this.bridge.unsubscribe(topic);
}

Connection.prototype.publish = function(topic, typeStr, payload) {
    if (!this.advertised[topic]) {
        this.advertised[topic] = true;
        this.bridge.advertise(topic, typeStr);
    }
    if (typeof payload == "string") {
        payload = JSON.parse(payload);
    }
    this.bridge.publish(topic, payload);
}

Connection.prototype.addHandler = function(topic, func) {
    if (this.bridge.subscription_handlers[topic] == null) {
        this.bridge.subscription_handlers[topic] = {};
    }
    if (this.bridge.subscription_handlers[topic][null] == null) {
        this.bridge.subscription_handlers[topic][null] = [];
    }
    this.bridge.subscription_handlers[topic][null].push(func);
}
    
Connection.prototype.removeHandler = function(topic) {
    if (this.bridge.subscription_handlers[topic] && this.bridge.subscription_handlers[topic][null]) {
        delete this.bridge.subscription_handlers[topic][null];
        if (Object.keys(this.bridge.subscription_handlers[topic]).length == 0) {
            delete this.bridge.subscription_handlers[topic];
        }
    }
}

Connection.prototype.setOnError = function(func) {
  this.bridge.onError = func;
}

Connection.prototype.setOnClose = function(func) {
  this.bridge.onClose = func;
}

Connection.prototype.setOnOpen = function(func) {
  this.bridge.onOpen = func;
}

Connection.prototype.setOnMessage = function(func) {
  this.bridge.onMessage = func;
}

ros.Connection = Connection;
ros.Bridge = Bridge;
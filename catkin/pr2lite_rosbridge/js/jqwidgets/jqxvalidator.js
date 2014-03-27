/*
jQWidgets v2.4.1 (2012-Sep-07)
Copyright (c) 2011-2012 jQWidgets.
License: http://jqwidgets.com/license/
*/

(function(a){a.jqx.jqxWidget("jqxValidator","",{});a.extend(a.jqx._jqxValidator.prototype,{defineInstance:function(){this.rules=null;this.scroll=true;this.focus=true;this.scrollDuration=300;this.scrollCallback=null;this.position="right";this.arrow=true;this.animation="fade";this.animationDuration=150;this.closeOnClick=true;this.onError=null;this.onSuccess=null;this.ownerElement=null;this._events=["validationError","validationSuccess"];this._margin=5;this._inputHint=[]},createInstance:function(){this._configureInputs();this._removeEventListeners();this._addEventListeners()},destroy:function(){this._removeEventListeners();this.hide()},validate:function(){var b=true,k,d=Infinity,g,f,c,h=[],j;this.updatePosition();for(var e=0;e<this.rules.length;e+=1){k=this._validateRule(this.rules[e]);if(!k){b=false;c=a(this.rules[e].input);h.push(c);g=c.offset().top;if(d>g){d=g;f=c}}}this._handleValidation(b,d,f,h);return b},validateInput:function(b){var e=this._getRulesForInput(b),d=true;for(var c=0;c<e.length;c+=1){if(!this._validateRule(e[c])){d=false}}return d},hideHint:function(b){var d=this._getRulesForInput(b);for(var c=0;c<d.length;c+=1){this._hideHintByRule(d[c])}},hide:function(){var c;for(var b=0;b<this.rules.length;b+=1){c=this.rules[b];this._hideHintByRule(this.rules[b])}},updatePosition:function(){var c;for(var b=0;b<this.rules.length;b+=1){c=this.rules[b];if(c.hint){this._hintLayout(c.hint,a(c.input),c.position)}}},_getRulesForInput:function(b){var d=[];for(var c=0;c<this.rules.length;c+=1){if(this.rules[c].input===b){d.push(this.rules[c])}}return d},_validateRule:function(d){var b=a(d.input),e,c=true;if(typeof d.rule==="function"&&!d.rule.call(this,b)){if(typeof d.hintRender==="function"&&!d.hint&&!this._higherPriorityActive(d)&&b.is(":visible")){e=d.hintRender.apply(this,[d.message,b]);this._hintLayout(e,b,d.position);this._showHint(e);d.hint=e;this._removeLowPriorityHints(d)}c=false}else{this._hideHintByRule(d)}return c},_hideHintByRule:function(c){var b=this,d;if(c){d=c.hint;if(d){if(this.animation==="fade"){d.fadeOut(this.animationDuration,function(){d.remove()})}else{d.remove()}}c.hint=null}},_handleValidation:function(b,e,d,c){if(!b){this._scrollHandler(e);if(this.focus){d.focus()}this._raiseEvent(0,{invalidInputs:c});if(typeof this.onError==="function"){this.onError(c)}}else{this._raiseEvent(1);if(typeof this.onSuccess==="function"){this.onSuccess()}}},_scrollHandler:function(c){if(this.scroll){var b=this;a("html,body").animate({scrollTop:c},this.scrollDuration,function(){if(typeof b.scrollCallback==="function"){b.scrollCallback.call(b)}})}},_higherPriorityActive:function(d){var e=false,c;for(var b=this.rules.length-1;b>=0;b-=1){c=this.rules[b];if(e&&c.input===d.input&&c.hint){return true}if(c===d){e=true}}return false},_removeLowPriorityHints:function(d){var e=false,c;for(var b=0;b<this.rules.length;b+=1){c=this.rules[b];if(e&&c.input===d.input){this._hideHintByRule(c)}if(c===d){e=true}}},_getHintRuleByInput:function(b){var d;for(var c=0;c<this.rules.length;c+=1){d=this.rules[c];if(a(d.input)[0]===b[0]&&d.hint){return d}}return null},_removeEventListeners:function(){var f,b,e;for(var d=0;d<this.rules.length;d+=1){f=this.rules[d];e=f.action.split(",");b=a(f.input);for(var c=0;c<e.length;c+=1){this.removeHandler(b,a.trim(e[c])+".jqx-validator")}}},_addEventListeners:function(){var f,c;if(this.host.parents(".jqx-window").length>0){var b=this;var g=function(){b.updatePosition()};var e=this.host.parents(".jqx-window");this.addHandler(e,"moved",g);this.addHandler(e,"moving",g);this.addHandler(e,"resized",g);this.addHandler(e,"resizing",g)}for(var d=0;d<this.rules.length;d+=1){f=this.rules[d];c=a(f.input);this._addListenerTo(c,f)}},_addListenerTo:function(c,h){var b=this,f=h.action.split(",");var e=false;if(this._isjQWidget(c)){e=true}for(var d=0;d<f.length;d+=1){var g=a.trim(f[d]);if(e&&(g=="blur"||g=="focus")){c=c.find("input")}this.addHandler(c,g+".jqx-validator",function(i){b._validateRule(h)})}},_configureInputs:function(){var b,d;this.rules=this.rules||[];for(var c=0;c<this.rules.length;c+=1){this._handleInput(c)}},_handleInput:function(b){var c=this.rules[b];if(!c.position){c.position=this.position}if(!c.message){c.message="test"}if(!c.action){c.action="blur"}if(!c.hintRender){c.hintRender=this._hintRender}if(!c.rule){c.rule=null}else{this._handleRule(c)}},_handleRule:function(f){var c=f.rule,e,d,b=false;if(typeof c==="string"){if(c.indexOf("=")>=0){c=c.split("=");d=c[1].split(",");c=c[0]}e=this["_"+c];if(e){f.rule=function(g){return e.apply(this,[g].concat(d))}}else{b=true}}else{if(typeof c!=="function"){b=true}else{f.rule=c}}if(b){throw new Error("Wrong parameter!")}},_required:function(b){switch(this._getType(b)){case"textarea":case"password":case"jqx-input":case"text":var d=a.data(b[0]);if(d.jqxMaskedInput){var e=b.jqxMaskedInput("promptChar"),c=b.jqxMaskedInput("value");return c&&c.indexOf(e)<0}else{if(d.jqxNumberInput){return b.jqxNumberInput("inputValue")!==""}else{if(d.jqxDateTimeInput){return true}else{return a.trim(b.val())!==""}}}case"checkbox":return b.is(":checked");case"radio":return b.is(":checked");case"div":if(b.is(".jqx-checkbox")){return b.jqxCheckBox("checked")}return false}return false},_notNumber:function(b){return this._validateText(b,function(d){if(d==""){return true}var c=/\d/;return !c.test(d)})},_number:function(b){return this._validateText(b,function(c){if(c==""){return true}return !isNaN(parseInt(c,10))})},_phone:function(b){return this._validateText(b,function(d){if(d==""){return true}var c=/^\(\d{3}\)(\d){3}-(\d){4}$/;return c.test(d)})},_length:function(c,d,b){return this._minLength(c,d)&&this._maxLength(c,b)},_maxLength:function(c,b){b=parseInt(b,10);return this._validateText(c,function(d){return d.length<=b})},_minLength:function(c,b){b=parseInt(b,10);return this._validateText(c,function(d){return d.length>=b})},_email:function(b){return this._validateText(b,function(d){if(d==""){return true}var c=/^(([^<>()[\]\\.,;:\s@\"]+(\.[^<>()[\]\\.,;:\s@\"]+)*)|(\".+\"))@((\[[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\])|(([a-zA-Z\-0-9]+\.)+[a-zA-Z]{2,}))$/;return c.test(d)})},_zipCode:function(b){return this._validateText(b,function(d){if(d==""){return true}var c=/^(^\d{5}$)|(^\d{5}-\d{4}$)|(\d{3}-\d{2}-\d{4})$/;return c.test(d)})},_ssn:function(b){return this._validateText(b,function(d){if(d==""){return true}var c=/\d{3}-\d{2}-\d{4}/;return c.test(d)})},_validateText:function(b,d){var c;if(this._isTextInput(b)){if(this._isjQWidget(b)){c=b.find("input").val()}else{c=b.val()}return d(c)}return false},_isjQWidget:function(b){var c=a.data(b[0]);if(c.jqxMaskedInput||c.jqxNumberInput||c.jqxDateTimeInput){return true}return false},_isTextInput:function(b){var c=this._getType(b);return c==="text"||c==="textarea"||c==="password"||b.is(".jqx-input")},_getType:function(c){var b=c[0].tagName.toLowerCase(),d;if(b==="textarea"){return"textarea"}else{if(c.is(".jqx-input")){return"jqx-input"}else{if(b==="input"){d=a(c).attr("type")?a(c).attr("type").toLowerCase():"text";return d}}}return b},_hintRender:function(d,c){var e=a('<div class="'+this.toThemeProperty("jqx-validator-hint")+' jqx-rc-all"></div>'),b=this;e.html(d);if(this.closeOnClick){e.click(function(){b.hideHint(c.selector)})}if(this.ownerElement==null){e.appendTo(document.body)}else{if(this.ownerElement.innerHTML){e.appendTo(a(this.ownerElement))}else{e.appendTo(this.ownerElement)}}return e},_hintLayout:function(d,c,b){var e;e=this._getPosition(c,b,d);d.css({position:"absolute",left:e.left,top:e.top});if(this.arrow){this._addArrow(c,d,b,e)}},_showHint:function(b){if(this.animation==="fade"){b.fadeOut(0);b.fadeIn(this.animationDuration)}},_getPosition:function(c,b,g){var f=c.offset(),e,d;if(this.ownerElement!=null){f={left:0,top:0};f.top=parseInt(f.top)+c.position().top;f.left=parseInt(f.left)+c.position().left}if(b.indexOf("top")>=0){e=f.top-c.outerHeight()}else{if(b.indexOf("bottom")>=0){e=f.top+g.outerHeight()+this._margin}else{e=f.top}}if(b.indexOf("center")>=0){d=f.left+this._margin+(c.outerWidth()-g.outerWidth())/2}else{if(b.indexOf("left")>=0){d=f.left-g.outerWidth()-this._margin}else{if(b.indexOf("right")>=0){d=f.left+c.outerWidth()+this._margin}else{d=f.left+this._margin}}}if(b.indexOf(":")>=0){b=b.split(":")[1].split(",");d+=parseInt(b[0],10);e+=parseInt(b[1],10)}return{left:d,top:e}},_addArrow:function(j,e,g,k){var l=a('<div class="'+this.toThemeProperty("jqx-validator-hint-arrow")+'"></div>'),d,i;e.children(".jqx-validator-hint-arrow").remove();e.append(l);var c=l.outerHeight(),f=l.outerWidth(),h=e.outerHeight(),b=e.outerWidth();this._addImage(l);if(g.indexOf("top")>=0){i=h-c}else{if(g.indexOf("bottom")>=0){i=-c}else{i=(h-c)/2-c/2}}if(g.indexOf("center")>=0){d=(b-f)/2}else{if(g.indexOf("left")>=0){d=b+f/2}else{if(g.indexOf("right")>=0){d=-f/2}}}if(g.indexOf("topright")>=0||g.indexOf("bottomright")>=0){d=0}if(g.indexOf("topleft")>=0||g.indexOf("bottomleft")>=0){d=b-f}l.css({position:"absolute",left:d,top:i})},_addImage:function(b){var c=b.css("background-image");c=c.replace('url("',"");c=c.replace('")',"");c=c.replace("url(","");c=c.replace(")","");b.css("background-image","none");b.append('<img src="'+c+'" alt="Arrow" style="position: relative; top: 0px; left: 0px; width: '+b.width()+"px; height: "+b.height()+'px;" />')},_raiseEvent:function(b,d){var c=a.Event(this._events[b]);c.args=d;return this.host.trigger(c)},propertyChangedHandler:function(b,c,e,d){if(c==="rules"){this._configureInputs();this._removeEventListeners();this._addEventListeners()}}})})(jQuery);
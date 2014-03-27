  var keyPressed= new Array();
  var keyCode;

  function bindKeyboardEvent(rosNode)
  {
    document.onkeydown = keydownevent;
    document.onkeyup = keyupevent;
    //       left, right,   up, down,  SP, Enter,  W,  A,  S,  D,  Q,  E,  I,  J,  K,  L,  U,  O
    keycode = [37,    39,    38,   40, 32,    13, 87, 65, 83, 68, 81, 69, 73, 74, 75, 76, 85, 79];

    for(var i = 0; i < keycode.length; i ++)
      keyPressed[i] = false;

    var publishKey = function() {

      var output = {};
      output.keys = keyPressed;
//      ros_debug(output);
      rosNode.publish("/keyboard","robot_booking/Keys",ros.json(output));
    }

    setInterval(publishKey, 200);

  }

  function keydownevent(e)
  {
    for(var i in keycode)
      if(keycode[i] == e.keyCode) {
        keyPressed[i] = true;
        break;
      }
  }

  function keyupevent(e)
  {
    for(var i in keycode)
      if(keycode[i] == e.keyCode) {
        keyPressed[i] = false;
        break;
      }
  }


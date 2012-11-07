"""autogenerated by genmsg_py from InteractiveBlockManipulationResult.msg. Do not edit."""
import roslib.message
import struct

import geometry_msgs.msg

class InteractiveBlockManipulationResult(roslib.message.Message):
  _md5sum = "3fec3f60e60c18ca7b67a7513b211e95"
  _type = "turtlebot_block_manipulation/InteractiveBlockManipulationResult"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
#result definition
geometry_msgs/Pose pickup_pose
geometry_msgs/Pose place_pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of postion and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

"""
  __slots__ = ['pickup_pose','place_pose']
  _slot_types = ['geometry_msgs/Pose','geometry_msgs/Pose']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       pickup_pose,place_pose
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(InteractiveBlockManipulationResult, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.pickup_pose is None:
        self.pickup_pose = geometry_msgs.msg.Pose()
      if self.place_pose is None:
        self.place_pose = geometry_msgs.msg.Pose()
    else:
      self.pickup_pose = geometry_msgs.msg.Pose()
      self.place_pose = geometry_msgs.msg.Pose()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_14d.pack(_x.pickup_pose.position.x, _x.pickup_pose.position.y, _x.pickup_pose.position.z, _x.pickup_pose.orientation.x, _x.pickup_pose.orientation.y, _x.pickup_pose.orientation.z, _x.pickup_pose.orientation.w, _x.place_pose.position.x, _x.place_pose.position.y, _x.place_pose.position.z, _x.place_pose.orientation.x, _x.place_pose.orientation.y, _x.place_pose.orientation.z, _x.place_pose.orientation.w))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.pickup_pose is None:
        self.pickup_pose = geometry_msgs.msg.Pose()
      if self.place_pose is None:
        self.place_pose = geometry_msgs.msg.Pose()
      end = 0
      _x = self
      start = end
      end += 112
      (_x.pickup_pose.position.x, _x.pickup_pose.position.y, _x.pickup_pose.position.z, _x.pickup_pose.orientation.x, _x.pickup_pose.orientation.y, _x.pickup_pose.orientation.z, _x.pickup_pose.orientation.w, _x.place_pose.position.x, _x.place_pose.position.y, _x.place_pose.position.z, _x.place_pose.orientation.x, _x.place_pose.orientation.y, _x.place_pose.orientation.z, _x.place_pose.orientation.w,) = _struct_14d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_14d.pack(_x.pickup_pose.position.x, _x.pickup_pose.position.y, _x.pickup_pose.position.z, _x.pickup_pose.orientation.x, _x.pickup_pose.orientation.y, _x.pickup_pose.orientation.z, _x.pickup_pose.orientation.w, _x.place_pose.position.x, _x.place_pose.position.y, _x.place_pose.position.z, _x.place_pose.orientation.x, _x.place_pose.orientation.y, _x.place_pose.orientation.z, _x.place_pose.orientation.w))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      if self.pickup_pose is None:
        self.pickup_pose = geometry_msgs.msg.Pose()
      if self.place_pose is None:
        self.place_pose = geometry_msgs.msg.Pose()
      end = 0
      _x = self
      start = end
      end += 112
      (_x.pickup_pose.position.x, _x.pickup_pose.position.y, _x.pickup_pose.position.z, _x.pickup_pose.orientation.x, _x.pickup_pose.orientation.y, _x.pickup_pose.orientation.z, _x.pickup_pose.orientation.w, _x.place_pose.position.x, _x.place_pose.position.y, _x.place_pose.position.z, _x.place_pose.orientation.x, _x.place_pose.orientation.y, _x.place_pose.orientation.z, _x.place_pose.orientation.w,) = _struct_14d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_14d = struct.Struct("<14d")

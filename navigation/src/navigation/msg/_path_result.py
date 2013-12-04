"""autogenerated by genpy from navigation/path_result.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class path_result(genpy.Message):
  _md5sum = "c2b5448f88283c1ef325611e59255ef3"
  _type = "navigation/path_result"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 x1
float64 y1
float64 x2
float64 y2
int64 edge_type

"""
  __slots__ = ['x1','y1','x2','y2','edge_type']
  _slot_types = ['float64','float64','float64','float64','int64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       x1,y1,x2,y2,edge_type

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(path_result, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.x1 is None:
        self.x1 = 0.
      if self.y1 is None:
        self.y1 = 0.
      if self.x2 is None:
        self.x2 = 0.
      if self.y2 is None:
        self.y2 = 0.
      if self.edge_type is None:
        self.edge_type = 0
    else:
      self.x1 = 0.
      self.y1 = 0.
      self.x2 = 0.
      self.y2 = 0.
      self.edge_type = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_4dq.pack(_x.x1, _x.y1, _x.x2, _x.y2, _x.edge_type))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 40
      (_x.x1, _x.y1, _x.x2, _x.y2, _x.edge_type,) = _struct_4dq.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_4dq.pack(_x.x1, _x.y1, _x.x2, _x.y2, _x.edge_type))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 40
      (_x.x1, _x.y1, _x.x2, _x.y2, _x.edge_type,) = _struct_4dq.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_4dq = struct.Struct("<4dq")
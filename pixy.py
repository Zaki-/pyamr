from pixy import *

pixy_init()

#Blocks
class Blocks (Structure):
      _fields_ = [ ("type", c_uint),
                   ("signature", c_uint),
                   ("x", c_uint),
                   ("y", c_uint),
                   ("width", c_uint),
                   ("height", c_uint),
                   ("angle", c_uint) ]

blocks = BlockArray(100)


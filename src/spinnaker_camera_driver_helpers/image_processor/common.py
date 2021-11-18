

class EncoderError(RuntimeError):
  def __init__(self, msg):
    super(EncoderError, self).__init__(msg)
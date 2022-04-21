class LCD:
	def __init__(self, name='Motion'):
		self.name = name
		self.debug = 0
		self.point = [1,2]
		self.tol = 40
		self.ps = None
		self.future=None
		self.last_cmd = None
		self.direction = 1

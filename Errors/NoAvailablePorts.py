class NoAvailablePorts(Exception):
	def __init__(self, message = "Nao Existem Portas Disponiveis"):
		self.message = message;
		super().__init__(self.message);
	

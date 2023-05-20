class PerformanceControl:
    def __init__(self):
        self.steps_count = 0

    def add_steps(self, count):
        self.steps_count += count

    def get_steps_count(self):
        return self.steps_count

from collections import deque

class HistoryBuffer():
    def __init__(self, queue_size: int):
        self.queue_size = queue_size
        self.queue = deque()
        self.count = 0
        self.total = 0

    def insert(self, item: float):
        # Remove item from queue if needed
        if (self.queue_size <= len(self.queue)):
            removed = self.queue.popleft()
            self.total -= removed

        self.queue.append(item)
        self.total += item

    def mean_value(self):
        if (len(self.queue) == 0): return 0

        return self.total / len(self.queue)
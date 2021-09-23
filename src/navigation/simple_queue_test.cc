#include <stdio.h>
#include <stdint.h>

#include "simple_queue.h"

int main() {
  // Construct the priority queue, to use uint64_t to represent node ID, and float as the priority type.
  SimpleQueue<uint64_t, float> queue;
  // Check if the queue is empty
  printf("Empty?: %d\n", queue.Empty());
  // Insert the node with ID 42 with priority 2.34 into the queue:
  queue.Push(42, 2.43);
  // Insert the node with ID 43 with priority 5.1 into the queue:
  queue.Push(43, 5.1);
  // Update the node with ID 42 with priority 2.1 into the queue:
  queue.Push(42, 2.1);
  // Check if the queue is empty
  printf("Empty?: %d\n", queue.Empty());
  // Get the priority element, this will return ID 42, since it has the better priority (2.1 < 5.1)
  uint64_t next_node = queue.Pop();
  printf("Next: %lu\n", next_node);
}

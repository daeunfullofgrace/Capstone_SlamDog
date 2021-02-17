#!/usr/bin/env python

class Node:
    def __init__(self, data):
        self.data = data
        self.next = None

class Stack:
    def __init__(self):
        self.head = None

    def is_empty(self):
        if not self.head:
            return True

        return False

    def push(self, data):
        new_node = Node(data)

        new_node.next = self.head
        self.head = new_node

    def pop(self):
        if self.is_empty():
            return None

        ret_data = self.head.data

        self.head = self.head.next

        return ret_data

    def peek(self):
        if self.is_empty():
            return None

        return self.head.data

# if __name__ == "main":
#     s = Stack()
#
#     print(s.is_empty()) # True
#
#     s.push(1)
#     s.push(2)
#     s.push(3)
#     s.push(4)
#     s.push(5)
#
#     print("peek of data : {}".format(s.peek())) # 5
#
#     while not s.is_empty():
#         print(s.pop()) # 5, 4, 3, 2, 1
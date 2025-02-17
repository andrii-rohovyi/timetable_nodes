class Node:
    def __init__(self, key, color='R'):
        self.key = key
        self.color = color  # 'R' for Red, 'B' for Black
        self.left = None
        self.right = None
        self.parent = None


class RedBlackTree:
    def __init__(self):
        self.NIL = Node(None, 'B')  # Sentinel NIL node
        self.root = self.NIL

    def insert(self, key):
        new_node = Node(key)
        new_node.left = new_node.right = self.NIL
        parent = None
        current = self.root

        while current != self.NIL:
            parent = current
            if key < current.key:
                current = current.left
            else:
                current = current.right

        new_node.parent = parent
        if parent is None:
            self.root = new_node
        elif key < parent.key:
            parent.left = new_node
        else:
            parent.right = new_node

        new_node.color = 'R'
        self._fix_insert(new_node)

    def _fix_insert(self, node):
        while node.parent and node.parent.color == 'R':
            if node.parent == node.parent.parent.left:
                uncle = node.parent.parent.right
                if uncle.color == 'R':
                    node.parent.color = 'B'
                    uncle.color = 'B'
                    node.parent.parent.color = 'R'
                    node = node.parent.parent
                else:
                    if node == node.parent.right:
                        node = node.parent
                        self._left_rotate(node)
                    node.parent.color = 'B'
                    node.parent.parent.color = 'R'
                    self._right_rotate(node.parent.parent)
            else:
                uncle = node.parent.parent.left
                if uncle.color == 'R':
                    node.parent.color = 'B'
                    uncle.color = 'B'
                    node.parent.parent.color = 'R'
                    node = node.parent.parent
                else:
                    if node == node.parent.left:
                        node = node.parent
                        self._right_rotate(node)
                    node.parent.color = 'B'
                    node.parent.parent.color = 'R'
                    self._left_rotate(node.parent.parent)
        self.root.color = 'B'

    def _left_rotate(self, node):
        right_child = node.right
        node.right = right_child.left
        if right_child.left != self.NIL:
            right_child.left.parent = node
        right_child.parent = node.parent
        if node.parent is None:
            self.root = right_child
        elif node == node.parent.left:
            node.parent.left = right_child
        else:
            node.parent.right = right_child
        right_child.left = node
        node.parent = right_child

    def _right_rotate(self, node):
        left_child = node.left
        node.left = left_child.right
        if left_child.right != self.NIL:
            left_child.right.parent = node
        left_child.parent = node.parent
        if node.parent is None:
            self.root = left_child
        elif node == node.parent.right:
            node.parent.right = left_child
        else:
            node.parent.left = left_child
        left_child.right = node
        node.parent = left_child

    def bisect_left(self, key):
        """Finds the leftmost index where `key` could be inserted."""
        current = self.root
        candidate = None
        while current != self.NIL:
            if current.key >= key:
                candidate = current
                current = current.left
            else:
                current = current.right
        return candidate.key if candidate else None

    def inorder(self, node=None, result=None):
        if result is None:
            result = []
        if node is None:
            node = self.root
        if node != self.NIL:
            self.inorder(node.left, result)
            result.append(node.key)
            self.inorder(node.right, result)
        return result
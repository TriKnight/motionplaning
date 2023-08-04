## This example code for Tree structure
class TreeNode:
    def __init__(self, value):
        self.value = value # data
        self.children = [] # references to other nodes

    def add_child(self, child_node):
        # creates parent-child relationship
        print("Adding ", child_node.value)
        self.children.append(child_node) 
    
    def remove_child(self, child_node):
        # removes parent-child relationship
        print("Removing " + child_node.value + " from " + self.value)
        self.children = [child for child in self.children 
                        if child is not child_node]

    def traverse(self):
        # moves through each node referenced from self downwards
        nodes_to_visit = [self]
        while len(nodes_to_visit) > 0:
            current_node = nodes_to_visit.pop()
            print(current_node.value)
            nodes_to_visit += current_node.children

   
## Test Tree structure
node0 = TreeNode(0)
node1 = TreeNode(1)
node2 = TreeNode(2)
node3 = TreeNode(3)
node4 = TreeNode(4)
node5 = TreeNode(5)
node6 = TreeNode(6)
node7 = TreeNode(7)
node8 = TreeNode(8)
## Define the node and it's values
node0.add_child(node1)
node1.add_child(node2)
node1.add_child(node5)
node1.add_child(node6)
node2.add_child(node3)
node2.add_child(node8)
node3.add_child(node4)
node6.add_child(node7)
for data in node1.children:
    print ("data in child in node 1: ", data.value)
print("show traverse data in the child node1: ")
node0.traverse()

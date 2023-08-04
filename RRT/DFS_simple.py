
#Code the Depth First Search
def dfs(graph, node, visited, component):
    component.append(node)  # Store answer
    visited[node] = True  # Mark visited

    # Traverse to each adjacent node of a node
    for child in graph[node]:
        if not visited[child]:  # Check whether the node is visited or not
            dfs(graph, child, visited, component)  # Call the dfs recursively


if __name__ == "__main__":
    # Graph of nodes with adjacency list 
    graph = {
        0: [1],
        1: [0, 2, 5, 6],
        2: [1, 8, 3],
        3: [2, 4],
        4: [3],
        5: [1],
        6: [1, 7],
        7: [6],
        8: [2]
    }
    node = 0  # Starting node
    visited = [False]*len(graph)  # Make all nodes to False initially
    
    component = []
    dfs(graph, node, visited, component)  # Traverse to each node of a graph
    print(f"Following is the Depth-first search: {component}")  # Print the answer
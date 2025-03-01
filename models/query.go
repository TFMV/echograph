package models

import (
	"fmt"
)

// PathResult represents a path between two nodes
type PathResult struct {
	Nodes []Node
	Edges []Edge
}

// NodeFilter is a function type used to filter nodes in queries
type NodeFilter func(node *Node) bool

// EdgeFilter is a function type used to filter edges in queries
type EdgeFilter func(edge *Edge) bool

// FindNodeByID returns a node by its ID
func (g *Graph) FindNodeByID(id string) (*Node, error) {
	for i, node := range g.Nodes {
		if node.ID == id {
			return &g.Nodes[i], nil
		}
	}
	return nil, fmt.Errorf("node with ID %s not found", id)
}

// FindEdgeByID returns an edge by its ID
func (g *Graph) FindEdgeByID(id string) (*Edge, error) {
	for i, edge := range g.Edges {
		if edge.ID == id {
			return &g.Edges[i], nil
		}
	}
	return nil, fmt.Errorf("edge with ID %s not found", id)
}

// FindNodesByType returns all nodes of a specific type
func (g *Graph) FindNodesByType(nodeType string) []Node {
	var result []Node
	for _, node := range g.Nodes {
		if node.Type == nodeType {
			result = append(result, node)
		}
	}
	return result
}

// FindEdgesByType returns all edges of a specific type
func (g *Graph) FindEdgesByType(edgeType string) []Edge {
	var result []Edge
	for _, edge := range g.Edges {
		if edge.Type == edgeType {
			result = append(result, edge)
		}
	}
	return result
}

// FindOutgoingEdges returns all edges originating from a node
func (g *Graph) FindOutgoingEdges(nodeID string) []Edge {
	var result []Edge
	for _, edge := range g.Edges {
		if edge.Source == nodeID {
			result = append(result, edge)
		}
	}
	return result
}

// FindIncomingEdges returns all edges targeting a node
func (g *Graph) FindIncomingEdges(nodeID string) []Edge {
	var result []Edge
	for _, edge := range g.Edges {
		if edge.Target == nodeID {
			result = append(result, edge)
		}
	}
	return result
}

// FindConnectedNodes returns all nodes directly connected to a node
func (g *Graph) FindConnectedNodes(nodeID string) []Node {
	var result []Node
	nodeMap := make(map[string]bool)

	// Find nodes connected by outgoing edges
	for _, edge := range g.Edges {
		if edge.Source == nodeID {
			nodeMap[edge.Target] = true
		}
		if edge.Target == nodeID {
			nodeMap[edge.Source] = true
		}
	}

	// Get the actual nodes
	for _, node := range g.Nodes {
		if nodeMap[node.ID] {
			result = append(result, node)
		}
	}

	return result
}

// FilterNodes returns nodes that match the provided filter function
func (g *Graph) FilterNodes(filter NodeFilter) []Node {
	var result []Node
	for i, node := range g.Nodes {
		if filter(&g.Nodes[i]) {
			result = append(result, node)
		}
	}
	return result
}

// FilterEdges returns edges that match the provided filter function
func (g *Graph) FilterEdges(filter EdgeFilter) []Edge {
	var result []Edge
	for i, edge := range g.Edges {
		if filter(&g.Edges[i]) {
			result = append(result, edge)
		}
	}
	return result
}

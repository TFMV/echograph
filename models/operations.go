package models

import (
	"fmt"
	"time"

	"github.com/google/uuid"
)

// NewNode creates a new node with a unique ID and timestamps
func NewNode(nodeType, label string, properties map[string]any) *Node {
	now := time.Now()
	id := uuid.New().String()
	return &Node{
		ID:         id,
		NodeID:     0, // This will need to be set by the caller based on requirements
		Type:       nodeType,
		Label:      label,
		Properties: properties,
		Size:       1.0,       // Default size
		Color:      "#808080", // Default color (gray)
		X:          0.0,       // Default X position
		Y:          0.0,       // Default Y position
		CreatedAt:  now,
		UpdatedAt:  now,
	}
}

// NewNodeWithID creates a new node with the specified NodeID and timestamps
func NewNodeWithID(nodeID int64, nodeType, label string, properties map[string]any) *Node {
	node := NewNode(nodeType, label, properties)
	node.NodeID = nodeID
	return node
}

// NewEdge creates a new edge with a unique ID and timestamps
func NewEdge(source, target, edgeType string, weight float64, properties map[string]any) *Edge {
	now := time.Now()
	return &Edge{
		ID:         uuid.New().String(),
		Source:     source,
		Target:     target,
		Type:       edgeType,
		Weight:     weight,
		EdgeWeight: weight,    // Initialize EdgeWeight to match Weight
		Color:      "#000000", // Default to black
		Style:      "solid",   // Default to solid line
		Properties: properties,
		CreatedAt:  now,
		UpdatedAt:  now,
	}
}

// SetNodePosition sets the position of a node
func (n *Node) SetPosition(x, y float64) {
	n.X = x
	n.Y = y
	n.UpdatedAt = time.Now()
}

// SetNodeAppearance sets the visual properties of a node
func (n *Node) SetAppearance(size float64, color string) {
	n.Size = size
	n.Color = color
	n.UpdatedAt = time.Now()
}

// NewGraph creates a new graph with a unique ID and timestamps
func NewGraph(name string) *Graph {
	now := time.Now()
	return &Graph{
		ID:                     uuid.New().String(),
		Name:                   name,
		Nodes:                  []Node{},
		Edges:                  []Edge{},
		Width:                  800,   // Default width
		Height:                 600,   // Default height
		MaxIterations:          1000,  // Default max iterations for physics
		StabilizationThreshold: 0.001, // Default stabilization threshold
		CreatedAt:              now,
		UpdatedAt:              now,
	}
}

// NewDataGraph creates a new data graph with the specified data source
func NewDataGraph(name, dataSource string) *DataGraph {
	g := NewGraph(name)
	return &DataGraph{
		Graph:                  g,
		DataSource:             dataSource,
		Metadata:               make(map[string]interface{}),
		Width:                  800,
		Height:                 600,
		Background:             "#ffffff",
		ForceStrength:          0.1,
		Gravity:                0.1,
		RepulsionForce:         100.0,
		DampingFactor:          0.9,
		SpringConstant:         0.04,
		SpringLength:           100.0,
		TimeStep:               0.5,
		MaxIterations:          g.MaxIterations,
		StabilizationThreshold: g.StabilizationThreshold,
	}
}

// AddNode adds a node to the graph
func (g *Graph) AddNode(node *Node) {
	g.Nodes = append(g.Nodes, *node)
	g.UpdatedAt = time.Now()
}

// AddEdge adds an edge to the graph
func (g *Graph) AddEdge(edge *Edge) error {
	// Check if source and target nodes exist
	sourceExists, targetExists := false, false
	var sourceNode, targetNode *Node

	for i, node := range g.Nodes {
		if node.ID == edge.Source {
			sourceExists = true
			sourceNode = &g.Nodes[i]
		}
		if node.ID == edge.Target {
			targetExists = true
			targetNode = &g.Nodes[i]
		}
		if sourceExists && targetExists {
			break
		}
	}

	if !sourceExists {
		return fmt.Errorf("source node with ID %s does not exist in the graph", edge.Source)
	}

	if !targetExists {
		return fmt.Errorf("target node with ID %s does not exist in the graph", edge.Target)
	}

	// Set the FromNode and ToNode references
	edge.FromNode = sourceNode
	edge.ToNode = targetNode

	g.Edges = append(g.Edges, *edge)
	g.UpdatedAt = time.Now()
	return nil
}

// RemoveNode removes a node and all connected edges from the graph
func (g *Graph) RemoveNode(nodeID string) {
	// Remove node
	var newNodes []Node
	for _, node := range g.Nodes {
		if node.ID != nodeID {
			newNodes = append(newNodes, node)
		}
	}
	g.Nodes = newNodes

	// Remove all edges connected to the node
	var newEdges []Edge
	for _, edge := range g.Edges {
		if edge.Source != nodeID && edge.Target != nodeID {
			newEdges = append(newEdges, edge)
		}
	}
	g.Edges = newEdges

	g.UpdatedAt = time.Now()
}

// RemoveEdge removes an edge from the graph
func (g *Graph) RemoveEdge(edgeID string) {
	var newEdges []Edge
	for _, edge := range g.Edges {
		if edge.ID != edgeID {
			newEdges = append(newEdges, edge)
		}
	}
	g.Edges = newEdges
	g.UpdatedAt = time.Now()
}

// SetDimensions sets the width and height of the graph
func (g *Graph) SetDimensions(width, height float64) {
	g.Width = width
	g.Height = height
	g.UpdatedAt = time.Now()
}

// SetPhysicsParameters sets the physics simulation parameters
func (g *Graph) SetPhysicsParameters(maxIterations int, stabilizationThreshold float64) {
	g.MaxIterations = maxIterations
	g.StabilizationThreshold = stabilizationThreshold
	g.UpdatedAt = time.Now()
}

// SetEdgeAppearance sets the visual properties of an edge
func (e *Edge) SetAppearance(color, style string) {
	e.Color = color
	e.Style = style
	e.UpdatedAt = time.Now()
}

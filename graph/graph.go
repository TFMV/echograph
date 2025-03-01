package graph

import (
	"sync"
)

type Node struct {
	ID     string
	Label  string
	X, Y   float64 // position for rendering
	VX, VY float64 // velocity used in physics simulation
	Mutex  sync.Mutex
}

type Edge struct {
	Source string
	Target string
	Weight float64
}

type Graph struct {
	Nodes map[string]*Node
	Edges []Edge
	Mutex sync.Mutex
}

// NewGraph creates an empty Graph.
func NewGraph() *Graph {
	return &Graph{
		Nodes: make(map[string]*Node),
		Edges: make([]Edge, 0),
	}
}

// AddNode inserts a new node into the graph.
func (g *Graph) AddNode(n Node) {
	g.Mutex.Lock()
	defer g.Mutex.Unlock()
	g.Nodes[n.ID] = &n
}

// AddEdge appends an edge to the graph.
func (g *Graph) AddEdge(e Edge) {
	g.Mutex.Lock()
	defer g.Mutex.Unlock()
	g.Edges = append(g.Edges, e)
}

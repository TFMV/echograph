// Package models provides data structures and interfaces for the echograph application.
// It defines the core domain models used throughout the application.
package models

import (
	"time"
)

// Node represents a node in the graph
type Node struct {
	ID         string         `json:"id"`
	NodeID     int64          `json:"node_id"` // Numeric identifier for physics calculations
	Type       string         `json:"type"`
	Label      string         `json:"label"`
	Data       interface{}    `json:"data,omitempty"`
	Size       float64        `json:"size"`
	Color      string         `json:"color"`
	X          float64        `json:"x"`
	Y          float64        `json:"y"`
	Properties map[string]any `json:"properties,omitempty"`
	CreatedAt  time.Time      `json:"created_at"`
	UpdatedAt  time.Time      `json:"updated_at"`
}

// Edge represents a directed edge between two nodes
type Edge struct {
	ID         string         `json:"id"`
	Source     string         `json:"source"`              // ID of the source node
	Target     string         `json:"target"`              // ID of the target node
	FromNode   *Node          `json:"from_node,omitempty"` // Reference to source node
	ToNode     *Node          `json:"to_node,omitempty"`   // Reference to target node
	Type       string         `json:"type"`
	Weight     float64        `json:"weight"`
	EdgeWeight float64        `json:"edge_weight"` // Alias for Weight for compatibility
	Color      string         `json:"color"`
	Style      string         `json:"style"` // e.g., "solid", "dashed", "dotted"
	Properties map[string]any `json:"properties,omitempty"`
	CreatedAt  time.Time      `json:"created_at"`
	UpdatedAt  time.Time      `json:"updated_at"`
}

// Graph represents a collection of nodes and edges
type Graph struct {
	ID                     string    `json:"id"`
	Name                   string    `json:"name"`
	Nodes                  []Node    `json:"nodes"`
	Edges                  []Edge    `json:"edges"`
	Width                  float64   `json:"width"`
	Height                 float64   `json:"height"`
	MaxIterations          int       `json:"max_iterations"`
	StabilizationThreshold float64   `json:"stabilization_threshold"`
	CreatedAt              time.Time `json:"created_at"`
	UpdatedAt              time.Time `json:"updated_at"`
}

// DataGraph is a specialized graph for data representation and visualization
type DataGraph struct {
	*Graph
	DataSource             string                 `json:"data_source"`
	Metadata               map[string]interface{} `json:"metadata,omitempty"`
	Width                  float64                `json:"width"`
	Height                 float64                `json:"height"`
	Background             string                 `json:"background"`
	ForceStrength          float64                `json:"force_strength"`
	Gravity                float64                `json:"gravity"`
	RepulsionForce         float64                `json:"repulsion_force"`
	DampingFactor          float64                `json:"damping_factor"`
	SpringConstant         float64                `json:"spring_constant"`
	SpringLength           float64                `json:"spring_length"`
	TimeStep               float64                `json:"time_step"`
	MaxIterations          int                    `json:"max_iterations"`
	StabilizationThreshold float64                `json:"stabilization_threshold"`
}

// NodeRepository defines operations for working with nodes
type NodeRepository interface {
	FindByID(id string) (*Node, error)
	FindByType(nodeType string) ([]*Node, error)
	Save(node *Node) error
	Delete(id string) error
}

// EdgeRepository defines operations for working with edges
type EdgeRepository interface {
	FindByID(id string) (*Edge, error)
	FindByNodes(sourceID, targetID string) ([]*Edge, error)
	Save(edge *Edge) error
	Delete(id string) error
}

// GraphRepository defines operations for working with graphs
type GraphRepository interface {
	FindByID(id string) (*Graph, error)
	FindByName(name string) ([]*Graph, error)
	Save(graph *Graph) error
	Delete(id string) error
}

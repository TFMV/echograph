package ingest

import (
	"encoding/csv"
	"encoding/json"
	"fmt"
	"io"
	"strings"

	"github.com/TFMV/echograph/models"
)

// DataProcessor defines the interface that all data processors must implement
type DataProcessor interface {
	// ProcessData takes raw data bytes and returns a graph representation
	ProcessData(data []byte) (*models.DataGraph, error)

	// GetName returns the name of the processor
	GetName() string
}

// Palette provides color schemes for graph visualization
type Palette struct {
	NodeColors []string
	EdgeColors []string
	Background string
}

// DefaultPalette returns a default color palette with vibrant colors
func DefaultPalette() *Palette {
	return &Palette{
		NodeColors: []string{
			"#4285F4", // Google Blue
			"#EA4335", // Google Red
			"#FBBC05", // Google Yellow
			"#34A853", // Google Green
			"#673AB7", // Purple
			"#3F51B5", // Indigo
			"#00BCD4", // Cyan
			"#009688", // Teal
			"#FF5722", // Deep Orange
		},
		EdgeColors: []string{
			"#666666", // Default gray
			"#888888", // Lighter gray
			"#AAAAAA", // Even lighter gray
		},
		Background: "#f8f8f8",
	}
}

// SurrealPalette returns a surrealist-inspired color palette
func SurrealPalette() *Palette {
	return &Palette{
		NodeColors: []string{
			"#FF6D00", // Amber
			"#2979FF", // Blue
			"#00E676", // Green
			"#F50057", // Pink
			"#651FFF", // Deep Purple
			"#C6FF00", // Lime
			"#FF3D00", // Deep Orange
			"#00B0FF", // Light Blue
			"#76FF03", // Light Green
		},
		EdgeColors: []string{
			"#333333", // Dark gray
			"#9C27B0", // Purple
			"#00BFA5", // Teal
		},
		Background: "#212121", // Dark background for contrast
	}
}

// JSONProcessor handles JSON data
type JSONProcessor struct {
	palette *Palette
}

// NewJSONProcessor creates a new JSON processor with the specified palette
func NewJSONProcessor(palette *Palette) *JSONProcessor {
	if palette == nil {
		palette = DefaultPalette()
	}
	return &JSONProcessor{palette: palette}
}

// GetName returns the name of the processor
func (p *JSONProcessor) GetName() string {
	return "JSON Processor"
}

// ProcessData processes JSON data
func (p *JSONProcessor) ProcessData(data []byte) (*models.DataGraph, error) {
	var graphData struct {
		Nodes []struct {
			ID    string                 `json:"id"`
			Label string                 `json:"label"`
			Data  map[string]interface{} `json:"data,omitempty"`
		} `json:"nodes"`
		Edges []struct {
			Source string  `json:"source"`
			Target string  `json:"target"`
			Weight float64 `json:"weight"`
		} `json:"edges"`
	}

	// Parse JSON data
	if err := json.Unmarshal(data, &graphData); err != nil {
		return nil, fmt.Errorf("error parsing JSON: %w", err)
	}

	// Create graph with high-quality defaults
	graph := models.NewDataGraph("JSON Import", "json")
	graph.Background = p.palette.Background
	graph.ForceStrength = 0.2
	graph.Gravity = 0.1
	graph.RepulsionForce = -800  // Stronger repulsion for better spacing
	graph.DampingFactor = 0.85   // Better damping for stable layouts
	graph.SpringConstant = 0.015 // Fine-tuned for natural edge shapes
	graph.SpringLength = 150     // More space between nodes
	graph.TimeStep = 0.5
	graph.MaxIterations = 500             // More iterations for better stabilization
	graph.StabilizationThreshold = 0.0005 // More strict stabilization
	graph.Width = 1200
	graph.Height = 900

	// Set the same values on the embedded Graph
	graph.Graph.Width = graph.Width
	graph.Graph.Height = graph.Height
	graph.Graph.MaxIterations = graph.MaxIterations
	graph.Graph.StabilizationThreshold = graph.StabilizationThreshold

	// Map to store nodes by their original IDs
	nodeMap := make(map[string]*models.Node)

	// Add nodes with artistic enhancements
	for i, n := range graphData.Nodes {
		// Select a color from the palette
		colorIndex := i % len(p.palette.NodeColors)

		// Vary the size based on the number of connections (calculated later)
		baseSize := 12.0

		node := models.NewNodeWithID(int64(i+1), "default", n.Label, nil)
		node.Data = n.Data
		node.Size = baseSize
		node.Color = p.palette.NodeColors[colorIndex]

		graph.Graph.AddNode(node)
		nodeMap[n.ID] = node
	}

	// Add edges with artistic styling
	for i, e := range graphData.Edges {
		source, sourceExists := nodeMap[e.Source]
		target, targetExists := nodeMap[e.Target]

		if !sourceExists || !targetExists {
			return nil, fmt.Errorf("edge references non-existent node: %s -> %s", e.Source, e.Target)
		}

		// Increase node size based on number of connections
		source.Size += 1.0
		target.Size += 1.0

		// Determine edge color from palette
		edgeColorIndex := i % len(p.palette.EdgeColors)
		edgeColor := p.palette.EdgeColors[edgeColorIndex]

		// For artistic variety, use different edge styles
		edgeStyle := "solid"
		if i%5 == 0 {
			edgeStyle = "dashed"
		} else if i%7 == 0 {
			edgeStyle = "dotted"
		}

		edge := models.NewEdge(source.ID, target.ID, "default", e.Weight, nil)
		edge.Color = edgeColor
		edge.Style = edgeStyle

		graph.Graph.AddEdge(edge)
	}

	// Normalize node sizes (min 8, max 24)
	minSize := 8.0
	maxSize := 24.0
	for i := range graph.Graph.Nodes {
		node := &graph.Graph.Nodes[i]
		if node.Size < minSize {
			node.Size = minSize
		} else if node.Size > maxSize {
			node.Size = maxSize
		}
	}

	return graph, nil
}

// CSVProcessor handles CSV data
type CSVProcessor struct {
	palette *Palette
}

// NewCSVProcessor creates a new CSV processor with the specified palette
func NewCSVProcessor(palette *Palette) *CSVProcessor {
	if palette == nil {
		palette = DefaultPalette()
	}
	return &CSVProcessor{palette: palette}
}

// GetName returns the name of the processor
func (p *CSVProcessor) GetName() string {
	return "CSV Processor"
}

// ProcessData processes CSV data
func (p *CSVProcessor) ProcessData(data []byte) (*models.DataGraph, error) {
	// Create a CSV reader
	reader := csv.NewReader(strings.NewReader(string(data)))

	// Create graph with high-quality defaults
	graph := models.NewDataGraph("CSV Import", "csv")
	graph.Background = p.palette.Background
	graph.ForceStrength = 0.2
	graph.Gravity = 0.1
	graph.RepulsionForce = -800
	graph.DampingFactor = 0.85
	graph.SpringConstant = 0.015
	graph.SpringLength = 150
	graph.TimeStep = 0.5
	graph.MaxIterations = 500
	graph.StabilizationThreshold = 0.0005
	graph.Width = 1200
	graph.Height = 900

	// Set the same values on the embedded Graph
	graph.Graph.Width = graph.Width
	graph.Graph.Height = graph.Height
	graph.Graph.MaxIterations = graph.MaxIterations
	graph.Graph.StabilizationThreshold = graph.StabilizationThreshold

	// Read header
	header, err := reader.Read()
	if err != nil {
		return nil, fmt.Errorf("error reading CSV header: %w", err)
	}

	// Find source and target columns
	sourceIdx, targetIdx := -1, -1
	weightIdx := -1
	labelIdx := -1

	for i, col := range header {
		colLower := strings.ToLower(col)
		switch {
		case colLower == "source" || colLower == "from" || colLower == "src":
			sourceIdx = i
		case colLower == "target" || colLower == "to" || colLower == "dst":
			targetIdx = i
		case colLower == "weight" || colLower == "value" || colLower == "strength":
			weightIdx = i
		case colLower == "label" || colLower == "name" || colLower == "title":
			labelIdx = i
		}
	}

	if sourceIdx == -1 || targetIdx == -1 {
		return nil, fmt.Errorf("CSV must contain source and target columns")
	}

	// Map to store nodes by their ID
	nodeMap := make(map[string]*models.Node)
	nodeCount := 0

	// Process rows
	for {
		row, err := reader.Read()
		if err == io.EOF {
			break
		}
		if err != nil {
			return nil, fmt.Errorf("error reading CSV row: %w", err)
		}

		sourceID := row[sourceIdx]
		targetID := row[targetIdx]

		// Create source node if it doesn't exist
		if _, exists := nodeMap[sourceID]; !exists {
			nodeCount++
			colorIdx := (nodeCount - 1) % len(p.palette.NodeColors)
			label := sourceID
			if labelIdx >= 0 && labelIdx < len(row) {
				label = row[labelIdx]
			}

			node := models.NewNodeWithID(int64(nodeCount), "default", label, nil)
			node.Size = 12.0
			node.Color = p.palette.NodeColors[colorIdx]

			graph.Graph.AddNode(node)
			nodeMap[sourceID] = node
		}

		// Create target node if it doesn't exist
		if _, exists := nodeMap[targetID]; !exists {
			nodeCount++
			colorIdx := (nodeCount - 1) % len(p.palette.NodeColors)
			label := targetID
			if labelIdx >= 0 && labelIdx < len(row) {
				label = row[labelIdx]
			}

			node := models.NewNodeWithID(int64(nodeCount), "default", label, nil)
			node.Size = 12.0
			node.Color = p.palette.NodeColors[colorIdx]

			graph.Graph.AddNode(node)
			nodeMap[targetID] = node
		}

		// Determine weight
		weight := 1.0
		if weightIdx >= 0 && weightIdx < len(row) {
			// Try to parse weight, default to 1.0 if parsing fails
			if w, err := fmt.Sscanf(row[weightIdx], "%f", &weight); err != nil || w != 1 {
				weight = 1.0
			}
		}

		// Create edge
		sourceNode := nodeMap[sourceID]
		targetNode := nodeMap[targetID]

		// Increase node size based on number of connections
		sourceNode.Size += 1.0
		targetNode.Size += 1.0

		edgeColorIdx := (nodeCount - 1) % len(p.palette.EdgeColors)
		edgeColor := p.palette.EdgeColors[edgeColorIdx]

		edge := models.NewEdge(sourceNode.ID, targetNode.ID, "default", weight, nil)
		edge.Color = edgeColor
		edge.Style = "solid"

		graph.Graph.AddEdge(edge)
	}

	// Normalize node sizes (min 8, max 24)
	minSize := 8.0
	maxSize := 24.0
	for i := range graph.Graph.Nodes {
		node := &graph.Graph.Nodes[i]
		if node.Size < minSize {
			node.Size = minSize
		} else if node.Size > maxSize {
			node.Size = maxSize
		}
	}

	return graph, nil
}

// LogProcessor handles log data
type LogProcessor struct {
	palette *Palette
}

// NewLogProcessor creates a new log processor with the specified palette
func NewLogProcessor(palette *Palette) *LogProcessor {
	if palette == nil {
		palette = DefaultPalette()
	}
	return &LogProcessor{palette: palette}
}

// GetName returns the name of the processor
func (p *LogProcessor) GetName() string {
	return "Log Processor"
}

// ProcessData processes log data
// For simplicity, logs are expected to be in a format where each line represents a relationship
// e.g., "A -> B", "X connected to Y", etc.
func (p *LogProcessor) ProcessData(data []byte) (*models.DataGraph, error) {
	lines := strings.Split(string(data), "\n")

	// Create graph with defaults
	graph := models.NewDataGraph("Log Import", "log")
	graph.Background = p.palette.Background
	graph.ForceStrength = 0.2
	graph.Gravity = 0.1
	graph.RepulsionForce = -800
	graph.DampingFactor = 0.85
	graph.SpringConstant = 0.015
	graph.SpringLength = 150
	graph.TimeStep = 0.5
	graph.MaxIterations = 500
	graph.StabilizationThreshold = 0.0005
	graph.Width = 1200
	graph.Height = 900

	// Set the same values on the embedded Graph
	graph.Graph.Width = graph.Width
	graph.Graph.Height = graph.Height
	graph.Graph.MaxIterations = graph.MaxIterations
	graph.Graph.StabilizationThreshold = graph.StabilizationThreshold

	// Map to store nodes by their ID
	nodeMap := make(map[string]*models.Node)
	nodeCount := 0
	edgeCount := 0

	// Common log patterns for connections
	patterns := []struct {
		separator     string
		bidirectional bool
	}{
		{" -> ", false},
		{" => ", false},
		{" connected to ", true},
		{" connects to ", false},
		{" links to ", false},
		{" linked to ", true},
		{" - ", true},
	}

	// Process each line
	for _, line := range lines {
		line = strings.TrimSpace(line)
		if line == "" {
			continue
		}

		// Try to match known patterns
		var sourceID, targetID string
		var found bool

		for _, pattern := range patterns {
			parts := strings.Split(line, pattern.separator)
			if len(parts) == 2 {
				sourceID = strings.TrimSpace(parts[0])
				targetID = strings.TrimSpace(parts[1])
				found = true
				break
			}
		}

		// Skip if no pattern matched
		if !found {
			continue
		}

		// Create source node if it doesn't exist
		if _, exists := nodeMap[sourceID]; !exists {
			nodeCount++
			colorIdx := (nodeCount - 1) % len(p.palette.NodeColors)

			node := models.NewNodeWithID(int64(nodeCount), "default", sourceID, nil)
			node.Size = 12.0
			node.Color = p.palette.NodeColors[colorIdx]

			graph.Graph.AddNode(node)
			nodeMap[sourceID] = node
		}

		// Create target node if it doesn't exist
		if _, exists := nodeMap[targetID]; !exists {
			nodeCount++
			colorIdx := (nodeCount - 1) % len(p.palette.NodeColors)

			node := models.NewNodeWithID(int64(nodeCount), "default", targetID, nil)
			node.Size = 12.0
			node.Color = p.palette.NodeColors[colorIdx]

			graph.Graph.AddNode(node)
			nodeMap[targetID] = node
		}

		// Create edge
		sourceNode := nodeMap[sourceID]
		targetNode := nodeMap[targetID]

		// Increase node size based on connections
		sourceNode.Size += 1.0
		targetNode.Size += 1.0

		edgeCount++
		edgeColorIdx := edgeCount % len(p.palette.EdgeColors)
		edgeColor := p.palette.EdgeColors[edgeColorIdx]

		edge := models.NewEdge(sourceNode.ID, targetNode.ID, "default", 1.0, nil)
		edge.Color = edgeColor
		edge.Style = "solid"

		graph.Graph.AddEdge(edge)
	}

	// Normalize node sizes
	minSize := 8.0
	maxSize := 24.0
	for i := range graph.Graph.Nodes {
		node := &graph.Graph.Nodes[i]
		if node.Size < minSize {
			node.Size = minSize
		} else if node.Size > maxSize {
			node.Size = maxSize
		}
	}

	return graph, nil
}

// GetProcessor returns the appropriate processor for the given format
func GetProcessor(format string) (DataProcessor, error) {
	switch strings.ToLower(format) {
	case "json":
		return NewJSONProcessor(DefaultPalette()), nil
	case "csv":
		return NewCSVProcessor(DefaultPalette()), nil
	case "log":
		return NewLogProcessor(DefaultPalette()), nil
	case "surreal-json":
		return NewJSONProcessor(SurrealPalette()), nil
	case "surreal-csv":
		return NewCSVProcessor(SurrealPalette()), nil
	case "surreal-log":
		return NewLogProcessor(SurrealPalette()), nil
	default:
		return nil, fmt.Errorf("unsupported format: %s", format)
	}
}

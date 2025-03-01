package render

import (
	"bytes"
	"encoding/json"
	"fmt"
	"math"
	"strings"
	"time"

	"github.com/TFMV/echograph/models"
	"github.com/TFMV/echograph/physics"
)

// OutputOptions defines rendering configuration options
type OutputOptions struct {
	Format         string  // Output format (svg, ascii, webgl, png, json, dot)
	Width          float64 // Width of the output
	Height         float64 // Height of the output
	Background     string  // Background color
	NoiseIntensity float64 // Intensity of surrealist noise (0.0-1.0)
	Timestamp      bool    // Include timestamp in visualization
	Animated       bool    // Create animated visualization (WebGL mode)
	AllowDragging  bool    // Allow node dragging (WebGL mode)
	AllowZooming   bool    // Allow zooming (WebGL mode)
	NodeSize       float64 // Default node size
	EdgeWidth      float64 // Default edge width
	FontSize       float64 // Font size for labels
	ShowLabels     bool    // Show node labels
	ShowEdgeLabels bool    // Show edge labels
	ColorScheme    string  // Color scheme (default, light, dark, surreal)
	Quality        string  // Rendering quality (low, medium, high)
}

// Renderer interface defines methods that all rendering backends must implement
type Renderer interface {
	// Render creates a visualization of the graph using the provided options
	Render(graph *models.Graph, options *OutputOptions) ([]byte, error)

	// Name returns the name of the renderer
	Name() string

	// Description returns a description of the renderer
	Description() string
}

// NewDefaultOptions creates a default set of output options
func NewDefaultOptions(format string) *OutputOptions {
	return &OutputOptions{
		Format:         format,
		Width:          800,
		Height:         600,
		Background:     "#f8f8f8",
		NoiseIntensity: 0.0,
		Timestamp:      true,
		Animated:       true,
		AllowDragging:  true,
		AllowZooming:   true,
		NodeSize:       12.0,
		EdgeWidth:      1.0,
		FontSize:       10.0,
		ShowLabels:     true,
		ShowEdgeLabels: false,
		ColorScheme:    "default",
		Quality:        "medium",
	}
}

// GetRenderer returns the appropriate renderer based on format
func GetRenderer(format string) (Renderer, error) {
	switch strings.ToLower(format) {
	case "svg":
		return &SVGRenderer{}, nil
	case "ascii":
		return &ASCIIRenderer{}, nil
	case "webgl":
		return &WebGLRenderer{}, nil
	case "json":
		return &JSONRenderer{}, nil
	case "dot":
		return &DOTRenderer{}, nil
	case "png":
		return &PNGRenderer{}, nil
	default:
		return nil, fmt.Errorf("unsupported output format: %s", format)
	}
}

// Generate processes a data graph and outputs a visualization
func Generate(graph *models.DataGraph, format string) ([]byte, error) {
	options := NewDefaultOptions(format)
	options.Width = graph.Width
	options.Height = graph.Height
	options.Background = graph.Background

	return GenerateWithOptions(graph, options)
}

// GenerateWithOptions processes a data graph with specific output options
func GenerateWithOptions(graph *models.DataGraph, options *OutputOptions) ([]byte, error) {
	// Create a channel to handle timeout
	done := make(chan struct{})
	errChan := make(chan error, 1)
	resultChan := make(chan []byte, 1)

	go func() {
		// Apply physics layout to position nodes
		layout := physics.NewForceDirectedLayout()

		// Apply surrealist distortion for more artistic effect if enabled
		var finalLayout physics.LayoutAlgorithm = layout

		if options.NoiseIntensity > 0 {
			surrealLayout := physics.NewSurrealLayout(layout)
			finalLayout = surrealLayout
		}

		// Initialize the layout
		finalLayout.Initialize(graph.Graph)

		// Run simulation steps until stable or max iterations reached
		// Use a limited number of iterations to prevent hanging
		maxSteps := graph.MaxIterations
		if maxSteps <= 0 {
			maxSteps = 100
		}

		for i := 0; i < maxSteps; i++ {
			if finalLayout.Step() {
				break
			}
		}

		// Apply final positions to graph
		finalLayout.Apply(graph.Graph)

		// Get appropriate renderer
		renderer, err := GetRenderer(options.Format)
		if err != nil {
			errChan <- err
			return
		}

		// Render the graph
		output, err := renderer.Render(graph.Graph, options)
		if err != nil {
			errChan <- err
			return
		}

		resultChan <- output
		close(done)
	}()

	// Set a timeout to prevent hanging
	select {
	case <-done:
		return <-resultChan, nil
	case err := <-errChan:
		return nil, err
	case <-time.After(30 * time.Second):
		return nil, fmt.Errorf("rendering timed out after 30 seconds")
	}
}

// SVGRenderer outputs SVG format
type SVGRenderer struct{}

// Name returns the name of the renderer
func (r *SVGRenderer) Name() string {
	return "SVG Renderer"
}

// Description returns a description of the renderer
func (r *SVGRenderer) Description() string {
	return "Renders graphs as Scalable Vector Graphics (SVG) for high-quality vector output"
}

// Render creates an SVG representation of the graph
func (r *SVGRenderer) Render(graph *models.Graph, options *OutputOptions) ([]byte, error) {
	var buf bytes.Buffer

	// SVG header with appropriate encoding and responsive viewBox
	buf.WriteString(fmt.Sprintf(`<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<svg width="%f" height="%f" viewBox="0 0 %f %f" xmlns="http://www.w3.org/2000/svg">
<rect width="100%%" height="100%%" fill="%s"/>
`, options.Width, options.Height, options.Width, options.Height, options.Background))

	// Add a defs section for styling and markers
	buf.WriteString(`<defs>
  <marker id="arrow" viewBox="0 0 10 10" refX="10" refY="5"
      markerWidth="6" markerHeight="6" orient="auto">
    <path d="M0,0 L10,5 L0,10 z" fill="#666666"/>
  </marker>
</defs>
`)

	// Draw a border if quality is high
	if options.Quality == "high" {
		buf.WriteString(fmt.Sprintf(`<rect x="0" y="0" width="%f" height="%f" fill="none" stroke="#e0e0e0" stroke-width="1"/>
`, options.Width, options.Height))
	}

	// Draw edges
	for _, edge := range graph.Edges {
		// Find the source and target nodes
		var sourceNode, targetNode *models.Node
		for i := range graph.Nodes {
			if graph.Nodes[i].ID == edge.Source {
				sourceNode = &graph.Nodes[i]
			}
			if graph.Nodes[i].ID == edge.Target {
				targetNode = &graph.Nodes[i]
			}
			if sourceNode != nil && targetNode != nil {
				break
			}
		}

		if sourceNode != nil && targetNode != nil {
			// Edge color and style properties
			edgeColor := "#666666"
			if edge.Color != "" {
				edgeColor = edge.Color
			}

			strokeWidth := options.EdgeWidth
			if edge.Weight > 0 {
				// Scale the edge width based on weight with a minimum
				strokeWidth = math.Max(0.5, edge.Weight*options.EdgeWidth*0.5)
			}

			dashArray := ""
			if edge.Style == "dashed" {
				dashArray = `stroke-dasharray="5,3"`
			} else if edge.Style == "dotted" {
				dashArray = `stroke-dasharray="1,3"`
			}

			// Draw the edge as a line
			buf.WriteString(fmt.Sprintf(`<line x1="%f" y1="%f" x2="%f" y2="%f" 
  stroke="%s" stroke-width="%f" %s />
`, sourceNode.X, sourceNode.Y, targetNode.X, targetNode.Y, edgeColor, strokeWidth, dashArray))

			// Add edge label if enabled
			if options.ShowEdgeLabels && edge.Type != "" {
				// Position label at the middle of the edge
				midX := (sourceNode.X + targetNode.X) / 2
				midY := (sourceNode.Y + targetNode.Y) / 2

				buf.WriteString(fmt.Sprintf(`<text x="%f" y="%f" font-family="sans-serif" font-size="%f" 
  fill="%s" text-anchor="middle" alignment-baseline="middle">%s</text>
`, midX, midY, options.FontSize, edgeColor, edge.Type))
			}
		}
	}

	// Draw nodes
	for _, node := range graph.Nodes {
		// Node color
		nodeColor := "#4285F4" // Default blue
		if node.Color != "" {
			nodeColor = node.Color
		}

		// Node size with fallback to options
		radius := node.Size
		if radius <= 0 {
			radius = options.NodeSize
		}

		// Draw shadow for high quality rendering
		if options.Quality == "high" {
			buf.WriteString(fmt.Sprintf(`<circle cx="%f" cy="%f" r="%f" fill="rgba(0,0,0,0.1)" transform="translate(2,2)" />
`, node.X, node.Y, radius))
		}

		// Draw the node as a circle
		buf.WriteString(fmt.Sprintf(`<circle cx="%f" cy="%f" r="%f" fill="%s" 
  stroke="rgba(0,0,0,0.3)" stroke-width="0.5" />
`, node.X, node.Y, radius, nodeColor))

		// Add node label if enabled
		if options.ShowLabels && node.Label != "" {
			labelY := node.Y + radius + options.FontSize + 2 // Position label below the node
			buf.WriteString(fmt.Sprintf(`<text x="%f" y="%f" font-family="sans-serif" font-size="%f" 
  fill="#333333" text-anchor="middle">%s</text>
`, node.X, labelY, options.FontSize, node.Label))
		}
	}

	// Add timestamp if requested
	if options.Timestamp {
		timeStr := time.Now().Format("2006-01-02 15:04:05")
		buf.WriteString(fmt.Sprintf(`<text x="5" y="%f" font-family="sans-serif" font-size="8" fill="#808080">%s</text>
`, options.Height-5, timeStr))
	}

	// Add graph metadata for high quality rendering
	if options.Quality == "high" {
		buf.WriteString(fmt.Sprintf(`<text x="5" y="15" font-family="sans-serif" font-size="10" fill="#808080">Nodes: %d | Edges: %d</text>
`, len(graph.Nodes), len(graph.Edges)))
	}

	// SVG footer
	buf.WriteString(`</svg>`)

	return buf.Bytes(), nil
}

// ASCIIRenderer outputs ASCII art format
type ASCIIRenderer struct{}

// Name returns the name of the renderer
func (r *ASCIIRenderer) Name() string {
	return "ASCII Renderer"
}

// Description returns a description of the renderer
func (r *ASCIIRenderer) Description() string {
	return "Renders graphs as ASCII art for terminal or text-based output"
}

// Render creates an ASCII representation of the graph
func (r *ASCIIRenderer) Render(graph *models.Graph, options *OutputOptions) ([]byte, error) {
	// Calculate dimensions based on options
	width := int(options.Width / 10)   // Scale down for ASCII
	height := int(options.Height / 20) // Scale down with adjustment for aspect ratio

	// Ensure minimum size
	width = max(width, 40)
	height = max(height, 20)

	// Create a grid for ASCII art
	grid := make([][]rune, height)
	for i := range grid {
		grid[i] = make([]rune, width)
		for j := range grid[i] {
			grid[i][j] = ' '
		}
	}

	// Draw a border around the graph
	for i := 0; i < width; i++ {
		grid[0][i] = '-'
		grid[height-1][i] = '-'
	}
	for i := 0; i < height; i++ {
		grid[i][0] = '|'
		grid[i][width-1] = '|'
	}
	grid[0][0] = '+'
	grid[0][width-1] = '+'
	grid[height-1][0] = '+'
	grid[height-1][width-1] = '+'

	// Draw edges
	for _, edge := range graph.Edges {
		var sourceNode, targetNode *models.Node

		// Find the connected nodes
		for i := range graph.Nodes {
			if graph.Nodes[i].ID == edge.Source {
				sourceNode = &graph.Nodes[i]
			}
			if graph.Nodes[i].ID == edge.Target {
				targetNode = &graph.Nodes[i]
			}
			if sourceNode != nil && targetNode != nil {
				break
			}
		}

		if sourceNode == nil || targetNode == nil {
			continue
		}

		// Convert floating point coordinates to grid indices
		x1 := int(sourceNode.X*float64(width-2)/options.Width) + 1
		y1 := int(sourceNode.Y*float64(height-2)/options.Height) + 1
		x2 := int(targetNode.X*float64(width-2)/options.Width) + 1
		y2 := int(targetNode.Y*float64(height-2)/options.Height) + 1

		// Ensure coordinates are in bounds
		x1 = clamp(x1, 1, width-2)
		y1 = clamp(y1, 1, height-2)
		x2 = clamp(x2, 1, width-2)
		y2 = clamp(y2, 1, height-2)

		// Draw line using a simple line algorithm
		drawLine(grid, x1, y1, x2, y2)
	}

	// Draw nodes
	nodeSymbols := []rune{'O', '@', '#', 'X', '*', '+'}
	for i, node := range graph.Nodes {
		x := int(node.X*float64(width-2)/options.Width) + 1
		y := int(node.Y*float64(height-2)/options.Height) + 1

		// Ensure coordinates are in bounds
		x = clamp(x, 1, width-2)
		y = clamp(y, 1, height-2)

		// Choose a symbol based on node index or attributes
		symbolIndex := i % len(nodeSymbols)
		nodeSymbol := nodeSymbols[symbolIndex]

		// Make sure coordinates are within bounds
		if x >= 0 && x < width && y >= 0 && y < height {
			grid[y][x] = nodeSymbol
		}

		// Add a simple label if enabled and space available
		if options.ShowLabels && node.Label != "" && y+1 < height && x < width {
			// Truncate label if too long
			label := node.Label
			if len(label) > width-x-1 {
				label = label[:width-x-1]
			}

			// Try to place the label
			if y+1 < height {
				for i := 0; i < len(label) && x+i < width-1; i++ {
					grid[y+1][x+i] = rune(label[i])
				}
			}
		}
	}

	// Add title and metadata at the top if there's space
	title := "EchoGraph - ASCII Rendering"
	if len(title) < width-4 && height > 3 {
		for i, c := range title {
			grid[1][i+2] = c
		}
	}

	// Add timestamp if requested
	if options.Timestamp && height > 4 {
		timeStr := time.Now().Format("2006-01-02 15:04")
		if len(timeStr) < width-4 {
			for i, c := range timeStr {
				grid[height-2][i+2] = c
			}
		}
	}

	// Convert grid to string
	var result strings.Builder
	for _, row := range grid {
		result.WriteString(string(row))
		result.WriteRune('\n')
	}

	return []byte(result.String()), nil
}

// WebGLRenderer outputs data for WebGL visualization
type WebGLRenderer struct{}

// Name returns the name of the renderer
func (r *WebGLRenderer) Name() string {
	return "WebGL Renderer"
}

// Description returns a description of the renderer
func (r *WebGLRenderer) Description() string {
	return "Renders interactive 3D visualizations using WebGL"
}

// Render creates a WebGL-ready representation of the graph
func (r *WebGLRenderer) Render(graph *models.Graph, options *OutputOptions) ([]byte, error) {
	// For WebGL, we'll generate both JSON data and HTML wrapper with JavaScript
	graphData, err := graphToJSON(graph)
	if err != nil {
		return nil, err
	}

	// Create HTML template with embedded graph data
	htmlStart := fmt.Sprintf(`<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>EchoGraph - Interactive Visualization</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/sigma.js/2.4.0/sigma.min.js"></script>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/graphology/0.25.4/graphology.umd.min.js"></script>
    <style>
        body, html { margin: 0; padding: 0; height: 100%%; overflow: hidden; }
        #graph-container { width: 100%%; height: 100%%; background: %s; }
    </style>
</head>
<body>
    <div id="graph-container"></div>
    <script>
    // Graph data
    const graphData = %s;
    
    // Create graph instance
    const graph = new graphology.Graph();
    
    // Add nodes
    graphData.nodes.forEach(node => {
        graph.addNode(node.id, {
            x: node.x,
            y: node.y,
            size: node.size,
            color: node.color,
            label: node.label
        });
    });
    
    // Add edges
    graphData.edges.forEach(edge => {
        graph.addEdge(edge.source, edge.target, {
            weight: edge.weight,
            color: edge.color
        });
    });
    
    // Initialize sigma`, options.Background, graphData)

	sigmaConfig := fmt.Sprintf(`    const renderer = new Sigma(graph, document.getElementById('graph-container'), {
        renderEdgeLabels: %v,
        labelRenderedSizeThreshold: 1,
        labelFont: 'Arial',
        labelSize: %f,
        defaultNodeColor: '#999',
        defaultEdgeColor: '#ccc',
        defaultNodeSize: %f,
        defaultEdgeSize: %f,
        allowDragging: %v,
        allowZoom: %v
    });`,
		options.ShowEdgeLabels,
		options.FontSize,
		options.NodeSize,
		options.EdgeWidth,
		options.AllowDragging,
		options.AllowZooming)

	animationCode := ""
	if options.Animated {
		animationCode = `
    // Animation logic
    let animationRunning = true;
    let frameId = null;
    
    function animate() {
        if (!animationRunning) return;
        
        // Apply subtle node movement
        graph.forEachNode((node, attributes) => {
            const noise = 0.5 * (Math.random() - 0.5);
            attributes.x += noise;
            attributes.y += noise;
        });
        
        renderer.refresh();
        frameId = requestAnimationFrame(animate);
    }
    
    // Start animation
    animate();
    
    // Toggle animation on click
    document.addEventListener('click', () => {
        animationRunning = !animationRunning;
        if (animationRunning) animate();
        else if (frameId) cancelAnimationFrame(frameId);
    });`
	}

	htmlEnd := `
    </script>
</body>
</html>`

	html := htmlStart + "\n" + sigmaConfig + "\n" + animationCode + htmlEnd

	return []byte(html), nil
}

// JSONRenderer outputs raw JSON format
type JSONRenderer struct{}

// Name returns the name of the renderer
func (r *JSONRenderer) Name() string {
	return "JSON Renderer"
}

// Description returns a description of the renderer
func (r *JSONRenderer) Description() string {
	return "Renders graph as JSON data for machine consumption or custom visualizations"
}

// Render creates a JSON representation of the graph
func (r *JSONRenderer) Render(graph *models.Graph, options *OutputOptions) ([]byte, error) {
	// Create structured data for JSON output
	type jsonNode struct {
		ID    string                 `json:"id"`
		Label string                 `json:"label"`
		X     float64                `json:"x"`
		Y     float64                `json:"y"`
		Size  float64                `json:"size"`
		Color string                 `json:"color"`
		Type  string                 `json:"type"`
		Data  map[string]interface{} `json:"data,omitempty"`
	}

	type jsonEdge struct {
		Source string  `json:"source"`
		Target string  `json:"target"`
		Weight float64 `json:"weight"`
		Color  string  `json:"color"`
		Style  string  `json:"style"`
		Type   string  `json:"type,omitempty"`
	}

	type jsonGraph struct {
		Nodes    []jsonNode             `json:"nodes"`
		Edges    []jsonEdge             `json:"edges"`
		Metadata map[string]interface{} `json:"metadata"`
	}

	// Convert graph to JSON structure
	jsonData := jsonGraph{
		Nodes: make([]jsonNode, 0, len(graph.Nodes)),
		Edges: make([]jsonEdge, 0, len(graph.Edges)),
		Metadata: map[string]interface{}{
			"width":      options.Width,
			"height":     options.Height,
			"background": options.Background,
			"timestamp":  time.Now().Format(time.RFC3339),
			"nodeCount":  len(graph.Nodes),
			"edgeCount":  len(graph.Edges),
		},
	}

	// Add nodes
	for _, node := range graph.Nodes {
		var nodeData map[string]interface{}
		if node.Data != nil {
			// Type assertion for node.Data
			if data, ok := node.Data.(map[string]interface{}); ok {
				nodeData = data
			}
		}

		jsonData.Nodes = append(jsonData.Nodes, jsonNode{
			ID:    node.ID,
			Label: node.Label,
			X:     node.X,
			Y:     node.Y,
			Size:  node.Size,
			Color: node.Color,
			Type:  node.Type,
			Data:  nodeData,
		})
	}

	// Add edges
	for _, edge := range graph.Edges {
		jsonData.Edges = append(jsonData.Edges, jsonEdge{
			Source: edge.Source,
			Target: edge.Target,
			Weight: edge.Weight,
			Color:  edge.Color,
			Style:  edge.Style,
			Type:   edge.Type,
		})
	}

	// Marshal to JSON
	return json.MarshalIndent(jsonData, "", "  ")
}

// DOTRenderer outputs Graphviz DOT format
type DOTRenderer struct{}

// Name returns the name of the renderer
func (r *DOTRenderer) Name() string {
	return "DOT Renderer"
}

// Description returns a description of the renderer
func (r *DOTRenderer) Description() string {
	return "Renders graph in Graphviz DOT format for compatibility with Graphviz tools"
}

// Render creates a DOT representation of the graph
func (r *DOTRenderer) Render(graph *models.Graph, options *OutputOptions) ([]byte, error) {
	var buf bytes.Buffer

	// Start the digraph
	buf.WriteString("digraph G {\n")

	// Graph attributes
	buf.WriteString(fmt.Sprintf("  graph [bgcolor=\"%s\", size=\"%f,%f\"];\n",
		options.Background, options.Width/72.0, options.Height/72.0))

	// Node default attributes
	buf.WriteString(fmt.Sprintf("  node [shape=circle, fontname=\"Arial\", fontsize=%f];\n",
		options.FontSize))

	// Edge default attributes
	buf.WriteString(fmt.Sprintf("  edge [fontname=\"Arial\", fontsize=%f];\n",
		options.FontSize*0.8))

	// Add nodes
	for _, node := range graph.Nodes {
		// Define node attributes
		color := node.Color
		if color == "" {
			color = "#4285F4"
		}

		size := node.Size
		if size <= 0 {
			size = options.NodeSize
		}

		label := node.Label
		if label == "" {
			label = node.ID
		}

		// Add node with attributes
		buf.WriteString(fmt.Sprintf("  \"%s\" [label=\"%s\", color=\"%s\", width=%f, pos=\"%f,%f!\"];\n",
			node.ID, label, color, size/20.0, node.X/100.0, node.Y/100.0))
	}

	// Add edges
	for _, edge := range graph.Edges {
		// Define edge attributes
		color := edge.Color
		if color == "" {
			color = "#666666"
		}

		weight := edge.Weight
		if weight <= 0 {
			weight = 1.0
		}

		style := edge.Style
		if style == "" {
			style = "solid"
		}

		// Add edge with attributes
		buf.WriteString(fmt.Sprintf("  \"%s\" -> \"%s\" [color=\"%s\", weight=%f, style=%s];\n",
			edge.Source, edge.Target, color, weight, style))
	}

	// End the digraph
	buf.WriteString("}\n")

	return buf.Bytes(), nil
}

// PNGRenderer uses SVG and converts to PNG
type PNGRenderer struct{}

// Name returns the name of the renderer
func (r *PNGRenderer) Name() string {
	return "PNG Renderer"
}

// Description returns a description of the renderer
func (r *PNGRenderer) Description() string {
	return "Renders graph as a PNG image (requires server-side rasterization)"
}

// Render creates a PNG representation of the graph
// This would normally require an image library to convert SVG to PNG
// For now, we return a message explaining this limitation
func (r *PNGRenderer) Render(graph *models.Graph, options *OutputOptions) ([]byte, error) {
	// First generate SVG
	svgRenderer := &SVGRenderer{}
	svg, err := svgRenderer.Render(graph, options)
	if err != nil {
		return nil, err
	}

	// Return the SVG with a comment explaining PNG conversion needs server-side processing
	msg := []byte("/* PNG rendering requires server-side processing with an image library. */\n")
	return append(msg, svg...), nil
}

// Helper functions

// Max returns the maximum of two integers
func max(a, b int) int {
	if a > b {
		return a
	}
	return b
}

// Convert a hex color string to an integer
func convertHexColor(hex string) uint32 {
	hex = strings.TrimPrefix(hex, "#")
	if len(hex) == 6 {
		var r, g, b uint32
		fmt.Sscanf(hex, "%02x%02x%02x", &r, &g, &b)
		return (r << 16) | (g << 8) | b
	}
	return 0
}

// Parse a hex color string into RGB components
func parseHexColor(hex string) (uint8, uint8, uint8) {
	// Make sure the hex string starts with #
	if !strings.HasPrefix(hex, "#") {
		hex = "#" + hex
	}

	// Remove the # prefix
	hex = strings.TrimPrefix(hex, "#")

	// Handle different formats
	if len(hex) == 3 {
		// Convert 3-digit hex to 6-digit
		r := parseHexDigit(hex[0])
		g := parseHexDigit(hex[1])
		b := parseHexDigit(hex[2])
		return r * 17, g * 17, b * 17 // Multiply by 17 to convert from 0-15 to 0-255
	} else if len(hex) >= 6 {
		// Parse 6-digit hex
		r := parseHexByte(hex[0:2])
		g := parseHexByte(hex[2:4])
		b := parseHexByte(hex[4:6])
		return r, g, b
	}

	// Default to black if invalid
	return 0, 0, 0
}

func parseHexDigit(c byte) uint8 {
	switch {
	case c >= '0' && c <= '9':
		return c - '0'
	case c >= 'a' && c <= 'f':
		return c - 'a' + 10
	case c >= 'A' && c <= 'F':
		return c - 'A' + 10
	}
	return 0
}

func parseHexByte(s string) uint8 {
	var result uint8
	for i := 0; i < len(s); i++ {
		result = result*16 + parseHexDigit(s[i])
	}
	return result
}

// Clamp a value between min and max
func clamp(val, min, max int) int {
	if val < min {
		return min
	}
	if val > max {
		return max
	}
	return val
}

// Draw a line on the ASCII grid using Bresenham's algorithm
func drawLine(grid [][]rune, x1, y1, x2, y2 int) {
	dx := abs(x2 - x1)
	dy := -abs(y2 - y1)
	sx := 1
	if x1 >= x2 {
		sx = -1
	}
	sy := 1
	if y1 >= y2 {
		sy = -1
	}
	err := dx + dy

	for {
		// Plot the point if it's in bounds
		if x1 >= 0 && x1 < len(grid[0]) && y1 >= 0 && y1 < len(grid) {
			// Don't overwrite node symbols
			if grid[y1][x1] != 'O' && grid[y1][x1] != '@' &&
				grid[y1][x1] != '#' && grid[y1][x1] != 'X' &&
				grid[y1][x1] != '*' && grid[y1][x1] != '+' {
				grid[y1][x1] = '·'
			}
		}

		if x1 == x2 && y1 == y2 {
			break
		}

		e2 := 2 * err
		if e2 >= dy {
			if x1 == x2 {
				break
			}
			err += dy
			x1 += sx
		}
		if e2 <= dx {
			if y1 == y2 {
				break
			}
			err += dx
			y1 += sy
		}
	}
}

// Absolute value of an integer
func abs(n int) int {
	if n < 0 {
		return -n
	}
	return n
}

// graphToJSON converts a graph to a JSON string for the WebGL renderer
func graphToJSON(graph *models.Graph) (string, error) {
	// Create structured data
	type jsonNode struct {
		ID    string  `json:"id"`
		Label string  `json:"label"`
		X     float64 `json:"x"`
		Y     float64 `json:"y"`
		Size  float64 `json:"size"`
		Color string  `json:"color"`
	}

	type jsonEdge struct {
		Source string  `json:"source"`
		Target string  `json:"target"`
		Weight float64 `json:"weight"`
		Color  string  `json:"color"`
	}

	type jsonGraph struct {
		Nodes []jsonNode `json:"nodes"`
		Edges []jsonEdge `json:"edges"`
	}

	// Convert graph to JSON structure
	jsonData := jsonGraph{
		Nodes: make([]jsonNode, 0, len(graph.Nodes)),
		Edges: make([]jsonEdge, 0, len(graph.Edges)),
	}

	// Add nodes
	for _, node := range graph.Nodes {
		color := node.Color
		if color == "" {
			color = "#4285F4"
		}

		size := node.Size
		if size <= 0 {
			size = 10.0
		}

		jsonData.Nodes = append(jsonData.Nodes, jsonNode{
			ID:    node.ID,
			Label: node.Label,
			X:     node.X,
			Y:     node.Y,
			Size:  size,
			Color: color,
		})
	}

	// Add edges
	for _, edge := range graph.Edges {
		color := edge.Color
		if color == "" {
			color = "#666666"
		}

		jsonData.Edges = append(jsonData.Edges, jsonEdge{
			Source: edge.Source,
			Target: edge.Target,
			Weight: edge.Weight,
			Color:  color,
		})
	}

	// Marshal to JSON
	jsonBytes, err := json.MarshalIndent(jsonData, "", "  ")
	if err != nil {
		return "", err
	}

	return string(jsonBytes), nil
}

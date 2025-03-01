package server

import (
	"encoding/json"
	"fmt"
	"io"
	"log"
	"net/http"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"time"

	"github.com/TFMV/echograph/ingest"
	"github.com/TFMV/echograph/models"
	"github.com/TFMV/echograph/render"
)

var (
	// Store any uploaded graphs in memory for demo purposes
	// In a production app, this would be a database
	graphs = make(map[string]*models.DataGraph)
)

// Configuration for the server
type Config struct {
	Port          int
	TemplatesPath string
	AssetsPath    string
	DebugMode     bool
}

// Start launches the web server on the specified port
func Start(port int) error {
	config := &Config{
		Port:          port,
		TemplatesPath: "assets/templates",
		AssetsPath:    "assets/static",
		DebugMode:     true,
	}

	// Ensure assets directory exists
	if err := ensureDirectories(config); err != nil {
		return err
	}

	// Register routes
	http.HandleFunc("/", handleIndex(config))
	http.HandleFunc("/upload", handleUpload(config))
	http.HandleFunc("/visualize", handleVisualize(config))
	http.HandleFunc("/api/graph", handleAPIGraph())

	// Serve static assets
	http.Handle("/static/", http.StripPrefix("/static/", http.FileServer(http.Dir(config.AssetsPath))))

	// Start server with timeout protection
	server := &http.Server{
		Addr:         fmt.Sprintf(":%d", config.Port),
		ReadTimeout:  10 * time.Second,
		WriteTimeout: 30 * time.Second,
		IdleTimeout:  120 * time.Second,
	}

	log.Printf("Starting server on port %d...", config.Port)
	return server.ListenAndServe()
}

// handleIndex renders the main page
func handleIndex(config *Config) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != "/" {
			http.NotFound(w, r)
			return
		}

		w.Header().Set("Content-Type", "text/html")
		fmt.Fprint(w, `
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>EchoGraph - Surrealist Data Visualization</title>
  <style>
    body {
      font-family: 'Helvetica Neue', Arial, sans-serif;
      margin: 0;
      padding: 20px;
      background: #f5f5f5;
      color: #333;
    }
    .container {
      max-width: 1200px;
      margin: 0 auto;
      background: white;
      padding: 30px;
      border-radius: 8px;
      box-shadow: 0 2px 10px rgba(0,0,0,0.1);
    }
    h1 {
      color: #2a2a2a;
      margin-top: 0;
      border-bottom: 2px solid #eee;
      padding-bottom: 10px;
    }
    .upload-section {
      margin: 20px 0;
      padding: 20px;
      background: #f9f9f9;
      border-radius: 4px;
    }
    .btn {
      background: #4285f4;
      color: white;
      border: none;
      padding: 10px 20px;
      border-radius: 4px;
      cursor: pointer;
      font-size: 16px;
    }
    .btn:hover {
      background: #3b78e7;
    }
    select, input {
      padding: 8px;
      font-size: 16px;
      border: 1px solid #ddd;
      border-radius: 4px;
      margin-right: 10px;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>EchoGraph: Surrealist Data Visualization</h1>
    
    <div class="upload-section">
      <h2>Upload Data</h2>
      <form action="/upload" method="post" enctype="multipart/form-data">
        <input type="file" name="dataFile" accept=".json,.csv,.log" required>
        <select name="outputFormat">
          <option value="svg">SVG (High Quality Vector)</option>
          <option value="webgl">WebGL (Interactive)</option>
          <option value="ascii">ASCII Art</option>
        </select>
        <button type="submit" class="btn">Visualize</button>
      </form>
    </div>
    
    <div class="sample-section">
      <h2>Sample Visualizations</h2>
      <a href="/visualize?id=sample1&format=svg" class="btn">Dreams Sample (SVG)</a>
      <a href="/visualize?id=sample1&format=webgl" class="btn">Dreams Sample (Interactive)</a>
    </div>
  </div>
</body>
</html>
`)
	}
}

// handleUpload processes uploaded data files
func handleUpload(config *Config) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		if r.Method != "POST" {
			http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
			return
		}

		// Parse multipart form
		err := r.ParseMultipartForm(10 << 20) // 10 MB limit
		if err != nil {
			http.Error(w, "Error parsing form: "+err.Error(), http.StatusBadRequest)
			return
		}

		// Get the file from the form
		file, handler, err := r.FormFile("dataFile")
		if err != nil {
			http.Error(w, "Error retrieving file: "+err.Error(), http.StatusBadRequest)
			return
		}
		defer file.Close()

		// Get output format
		format := r.FormValue("outputFormat")
		if format == "" {
			format = "svg" // Default format
		}

		// Create a temp file to store the upload
		tempFile, err := os.CreateTemp("", "upload-*"+filepath.Ext(handler.Filename))
		if err != nil {
			http.Error(w, "Error creating temp file: "+err.Error(), http.StatusInternalServerError)
			return
		}
		defer os.Remove(tempFile.Name())
		defer tempFile.Close()

		// Copy the uploaded file to the temp file
		_, err = io.Copy(tempFile, file)
		if err != nil {
			http.Error(w, "Error saving file: "+err.Error(), http.StatusInternalServerError)
			return
		}

		// Process the file
		graph, err := ProcessFile(tempFile.Name())
		if err != nil {
			http.Error(w, "Error processing file: "+err.Error(), http.StatusInternalServerError)
			return
		}

		// Store the graph in memory
		graphID := fmt.Sprintf("graph-%d", time.Now().Unix())
		graphs[graphID] = graph

		// Redirect to visualization
		http.Redirect(w, r, fmt.Sprintf("/visualize?id=%s&format=%s", graphID, format), http.StatusSeeOther)
	}
}

// handleVisualize renders the visualization
func handleVisualize(config *Config) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		// Get graph ID and format from query params
		graphID := r.URL.Query().Get("id")
		format := r.URL.Query().Get("format")

		if graphID == "" {
			http.Error(w, "Missing graph ID", http.StatusBadRequest)
			return
		}

		if format == "" {
			format = "svg" // Default format
		}

		// Check if this is a sample request
		var graph *models.DataGraph
		var err error

		if graphID == "sample1" {
			// Use the sample dreams graph
			graph, err = createSampleGraph()
			if err != nil {
				http.Error(w, "Error creating sample graph: "+err.Error(), http.StatusInternalServerError)
				return
			}
		} else {
			// Get the stored graph
			var exists bool
			graph, exists = graphs[graphID]
			if !exists {
				http.Error(w, "Graph not found", http.StatusNotFound)
				return
			}
		}

		// Apply high-quality settings
		width := 1200
		height := 900
		if r.URL.Query().Get("width") != "" {
			if w, err := strconv.Atoi(r.URL.Query().Get("width")); err == nil && w > 0 {
				width = w
			}
		}
		if r.URL.Query().Get("height") != "" {
			if h, err := strconv.Atoi(r.URL.Query().Get("height")); err == nil && h > 0 {
				height = h
			}
		}

		// Update graph dimensions for high quality
		graph.Width = float64(width)
		graph.Height = float64(height)

		// Set output options
		options := &render.OutputOptions{
			Format:         format,
			Width:          float64(width),
			Height:         float64(height),
			Background:     "#f8f8f8",
			NoiseIntensity: 0.5,
			Timestamp:      true,
			Animated:       true,
			AllowDragging:  true,
			AllowZooming:   true,
		}

		// Render the graph
		output, err := render.GenerateWithOptions(graph, options)
		if err != nil {
			http.Error(w, "Error generating visualization: "+err.Error(), http.StatusInternalServerError)
			return
		}

		// Set appropriate content type
		switch format {
		case "svg":
			w.Header().Set("Content-Type", "image/svg+xml")
		case "webgl":
			w.Header().Set("Content-Type", "text/html")
		case "ascii":
			w.Header().Set("Content-Type", "text/plain")
		}

		// Write the output
		w.Write(output)
	}
}

// handleAPIGraph provides a JSON API for the graph data
func handleAPIGraph() http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		graphID := r.URL.Query().Get("id")

		if graphID == "" {
			http.Error(w, "Missing graph ID", http.StatusBadRequest)
			return
		}

		// Check if this is a sample request
		var graph *models.DataGraph
		var err error

		if graphID == "sample1" {
			// Use the sample dreams graph
			graph, err = createSampleGraph()
			if err != nil {
				http.Error(w, "Error creating sample graph: "+err.Error(), http.StatusInternalServerError)
				return
			}
		} else {
			// Get the stored graph
			var exists bool
			graph, exists = graphs[graphID]
			if !exists {
				http.Error(w, "Graph not found", http.StatusNotFound)
				return
			}
		}

		// Set the content type
		w.Header().Set("Content-Type", "application/json")

		// Encode the graph as JSON
		encoder := json.NewEncoder(w)
		encoder.SetIndent("", "  ")
		encoder.Encode(graph)
	}
}

// createSampleGraph creates a sample graph for demonstration
func createSampleGraph() (*models.DataGraph, error) {
	// Sample JSON data representing the dreams graph
	data := []byte(`{
		"nodes": [
			{"id": "1", "label": "Dreams"},
			{"id": "2", "label": "Illusions"},
			{"id": "3", "label": "Memories"},
			{"id": "4", "label": "Reality"},
			{"id": "5", "label": "Surrealism"}
		],
		"edges": [
			{"source": "1", "target": "2", "weight": 1.2},
			{"source": "2", "target": "3", "weight": 0.9},
			{"source": "3", "target": "4", "weight": 1.0},
			{"source": "4", "target": "5", "weight": 1.5},
			{"source": "5", "target": "1", "weight": 2.0},
			{"source": "1", "target": "5", "weight": 1.8},
			{"source": "2", "target": "5", "weight": 0.7}
		]
	}`)

	// Process the sample data
	processor := &ingest.JSONProcessor{}
	return processor.ProcessData(data)
}

// ensureDirectories makes sure required directories exist
func ensureDirectories(config *Config) error {
	dirs := []string{
		config.TemplatesPath,
		config.AssetsPath,
	}

	for _, dir := range dirs {
		if err := os.MkdirAll(dir, 0755); err != nil {
			return fmt.Errorf("failed to create directory %s: %w", dir, err)
		}
	}

	return nil
}

// ProcessFile processes a file and returns a graph
func ProcessFile(filename string) (*models.DataGraph, error) {
	// Get the file extension
	ext := strings.ToLower(filepath.Ext(filename))

	// Read the file
	data, err := os.ReadFile(filename)
	if err != nil {
		return nil, fmt.Errorf("error reading file: %w", err)
	}

	// Process based on file type
	switch ext {
	case ".json":
		processor := &ingest.JSONProcessor{}
		return processor.ProcessData(data)
	default:
		return nil, fmt.Errorf("unsupported file format: %s", ext)
	}
}

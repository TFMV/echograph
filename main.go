package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"
	"path/filepath"
	"strings"
	"sync"
	"syscall"
	"time"

	"github.com/TFMV/echograph/ingest"
	"github.com/TFMV/echograph/models"
	"github.com/TFMV/echograph/physics"
	"github.com/TFMV/echograph/render"
	"github.com/TFMV/echograph/server"
)

// Configuration represents all the settings for the application
type Configuration struct {
	Mode           string
	DataFile       string
	OutputFile     string
	Port           int
	Width          float64
	Height         float64
	NoiseIntensity float64
	Timestamp      bool
	Animated       bool
	AllowDragging  bool
	AllowZooming   bool
	DebugMode      bool
	MaxIterations  int
}

func main() {
	// Create a context that can be canceled on SIGINT/SIGTERM
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	// Handle OS signals for graceful shutdown
	sig := make(chan os.Signal, 1)
	signal.Notify(sig, syscall.SIGINT, syscall.SIGTERM)
	go func() {
		<-sig
		log.Println("Received shutdown signal, gracefully shutting down...")
		cancel()
	}()

	// Parse configuration from command-line flags
	config := parseConfig()

	// Set up logging
	if config.DebugMode {
		log.SetFlags(log.LstdFlags | log.Lshortfile | log.Lmicroseconds)
		log.Println("Debug mode enabled")
	} else {
		log.SetFlags(log.LstdFlags)
	}

	// Process the input file
	graph, err := processInputFile(config.DataFile)
	if err != nil {
		log.Fatalf("Failed to process input file: %v", err)
	}

	// Apply physics layout
	if err := applyPhysicsLayout(ctx, graph, config); err != nil {
		log.Fatalf("Failed to apply physics layout: %v", err)
	}

	// Generate output
	if config.Mode == "server" {
		// Start the server mode
		if err := server.Start(config.Port); err != nil {
			log.Fatalf("Server failed: %v", err)
		}
	} else {
		// Render output
		if err := renderOutput(graph, config); err != nil {
			log.Fatalf("Rendering failed: %v", err)
		}
		log.Printf("Processing complete. Output saved to %s", config.OutputFile)
	}
}

// parseConfig parses command-line flags and returns a Configuration object
func parseConfig() *Configuration {
	config := &Configuration{}

	// Basic options
	flag.StringVar(&config.Mode, "mode", "svg", "Render mode: svg, webgl, ascii, server")
	flag.StringVar(&config.DataFile, "data", "", "Path to data file (JSON, CSV, SQL)")
	flag.StringVar(&config.OutputFile, "output", "", "Path to output file (defaults to 'output.[format]')")
	flag.IntVar(&config.Port, "port", 8080, "Port for server mode")

	// Visualization options
	flag.Float64Var(&config.Width, "width", 800.0, "Width of the visualization")
	flag.Float64Var(&config.Height, "height", 600.0, "Height of the visualization")
	flag.Float64Var(&config.NoiseIntensity, "noise", 0.0, "Intensity of surrealist noise (0.0-1.0)")
	flag.BoolVar(&config.Timestamp, "timestamp", true, "Include timestamp in visualization")
	flag.BoolVar(&config.Animated, "animated", true, "Create animated visualization (WebGL mode)")
	flag.BoolVar(&config.AllowDragging, "dragging", true, "Allow node dragging (WebGL mode)")
	flag.BoolVar(&config.AllowZooming, "zooming", true, "Allow zooming (WebGL mode)")

	// Advanced options
	flag.BoolVar(&config.DebugMode, "debug", false, "Enable debug logging")
	flag.IntVar(&config.MaxIterations, "iterations", 1000, "Maximum iterations for physics simulation")

	flag.Parse()

	// Validate required flags
	if config.DataFile == "" && config.Mode != "server" {
		fmt.Println("Please provide a data file using -data flag")
		flag.Usage()
		os.Exit(1)
	}

	// Set default output file if not specified
	if config.OutputFile == "" {
		switch config.Mode {
		case "svg":
			config.OutputFile = "output.svg"
		case "webgl":
			config.OutputFile = "output.html"
		case "ascii":
			config.OutputFile = "output.txt"
		}
	}

	return config
}

// processInputFile reads and processes the input file based on its extension
func processInputFile(dataFile string) (*models.Graph, error) {
	if dataFile == "" {
		return models.NewGraph("Empty Graph"), nil
	}

	// Determine the file type based on extension
	fileExt := strings.ToLower(filepath.Ext(dataFile))

	// Read the file
	data, err := os.ReadFile(dataFile)
	if err != nil {
		return nil, fmt.Errorf("failed to read file: %w", err)
	}

	// Process the file based on its type
	var dataGraph *models.DataGraph

	switch fileExt {
	case ".json":
		processor := ingest.NewJSONProcessor(nil)
		dataGraph, err = processor.ProcessData(data)
	case ".csv":
		return nil, fmt.Errorf("CSV processing not implemented yet")
	case ".sql":
		return nil, fmt.Errorf("SQL processing not implemented yet")
	case ".log":
		return nil, fmt.Errorf("log processing not implemented yet")
	default:
		return nil, fmt.Errorf("unsupported file type: %s", fileExt)
	}

	if err != nil {
		return nil, fmt.Errorf("failed to process data: %w", err)
	}

	return dataGraph.Graph, nil
}

// applyPhysicsLayout applies the physics simulation to position nodes
func applyPhysicsLayout(ctx context.Context, graph *models.Graph, config *Configuration) error {
	// Create layout algorithm
	layout := physics.NewForceDirectedLayout()

	// Apply surrealist distortion if noise intensity > 0
	var finalLayout physics.LayoutAlgorithm = layout
	if config.NoiseIntensity > 0 {
		surrealLayout := physics.NewSurrealLayout(layout)
		finalLayout = surrealLayout
	}

	// Initialize layout
	finalLayout.Initialize(graph)

	// Use WaitGroup to ensure layout completes before returning
	var wg sync.WaitGroup
	wg.Add(1)

	// Track completion status
	completed := false

	// Start physics simulation in a goroutine
	go func() {
		defer wg.Done()

		// Run simulation steps until stable or max iterations reached
		for i := 0; i < config.MaxIterations; i++ {
			select {
			case <-ctx.Done():
				// Context canceled, exit early
				return
			default:
				if finalLayout.Step() {
					completed = true
					break // Layout is stable
				}
			}
		}

		// Apply final positions to graph
		finalLayout.Apply(graph)
	}()

	// Wait for layout to complete with timeout
	done := make(chan struct{})
	go func() {
		wg.Wait()
		close(done)
	}()

	select {
	case <-done:
		if !completed && ctx.Err() == nil {
			log.Println("Warning: Physics simulation did not fully stabilize")
		}
		return nil
	case <-time.After(30 * time.Second):
		if ctx.Err() == nil {
			log.Println("Warning: Physics simulation timeout, using partial results")
			// Apply whatever progress we have
			finalLayout.Apply(graph)
			return nil
		}
		return ctx.Err()
	}
}

// renderOutput renders the graph using the specified renderer
func renderOutput(graph *models.Graph, config *Configuration) error {
	// Setup output options
	options := &render.OutputOptions{
		Format:         config.Mode,
		Width:          config.Width,
		Height:         config.Height,
		NoiseIntensity: config.NoiseIntensity,
		Timestamp:      config.Timestamp,
		Animated:       config.Animated,
		AllowDragging:  config.AllowDragging,
		AllowZooming:   config.AllowZooming,
	}

	var renderer render.Renderer
	var err error

	// Create the appropriate renderer
	switch config.Mode {
	case "svg":
		renderer = &render.SVGRenderer{}
	case "webgl":
		renderer = &render.WebGLRenderer{}
	case "ascii":
		renderer = &render.ASCIIRenderer{}
	case "json":
		renderer = &render.JSONRenderer{}
	case "dot":
		renderer = &render.DOTRenderer{}
	case "png":
		renderer = &render.PNGRenderer{}
	default:
		return fmt.Errorf("unsupported output format: %s", config.Mode)
	}

	// Render graph
	output, err := renderer.Render(graph, options)
	if err != nil {
		return fmt.Errorf("rendering failed: %w", err)
	}

	// Write output to file
	err = os.WriteFile(config.OutputFile, output, 0644)
	if err != nil {
		return fmt.Errorf("failed to write output file: %w", err)
	}

	return nil
}

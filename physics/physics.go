package physics

import (
	"math"
	"sync"
	"time"

	"github.com/TFMV/echograph/models"
	opensimplex "github.com/ojrac/opensimplex-go"
)

// LayoutAlgorithm defines an interface for layout algorithms
type LayoutAlgorithm interface {
	Initialize(graph *models.Graph)
	Step() bool // Returns true if stable, false if needs more steps
	Apply(graph *models.Graph)
	GetName() string
}

// ForceDirectedLayout implements a Fruchterman-Reingold force-directed layout
type ForceDirectedLayout struct {
	width           float64
	height          float64
	nodePositions   map[int64]position
	nodeVelocities  map[int64]velocity
	forces          map[int64]force
	temperature     float64
	k               float64 // optimal distance
	iterations      int
	maxIterations   int
	stable          bool
	energyThreshold float64
	mu              sync.Mutex
	edges           map[int64]map[int64][]*models.Edge // Cached edges for quick lookup
	gravity         float64                            // Gravity factor
	repulsionForce  float64                            // Repulsion strength
	dampingFactor   float64                            // Damping for velocity
	springConstant  float64                            // Spring stiffness
}

// Force vector components
type force struct {
	fx, fy float64
}

// Position coordinates
type position struct {
	x, y float64
}

// Velocity vector components
type velocity struct {
	vx, vy float64
}

// NewForceDirectedLayout creates a new force-directed layout
func NewForceDirectedLayout() *ForceDirectedLayout {
	return &ForceDirectedLayout{
		width:           800,
		height:          600,
		nodePositions:   make(map[int64]position),
		nodeVelocities:  make(map[int64]velocity),
		forces:          make(map[int64]force),
		edges:           make(map[int64]map[int64][]*models.Edge),
		temperature:     1.0,
		maxIterations:   1000,
		energyThreshold: 0.001,
		gravity:         0.05,
		repulsionForce:  100.0,
		dampingFactor:   0.9,
		springConstant:  0.04,
	}
}

// GetName returns the name of the layout algorithm
func (fd *ForceDirectedLayout) GetName() string {
	return "Force-Directed Layout"
}

// Initialize sets up the layout algorithm
func (fd *ForceDirectedLayout) Initialize(graph *models.Graph) {
	fd.mu.Lock()
	defer fd.mu.Unlock()

	fd.width = graph.Width
	fd.height = graph.Height
	fd.maxIterations = graph.MaxIterations
	fd.energyThreshold = graph.StabilizationThreshold

	// Use default physics parameters
	// Note: In a real implementation, we'd get these from the parent DataGraph
	// but we don't have direct access to it from the Graph struct

	// Optimal distance between nodes
	// Based on the size of the display area and number of nodes
	area := fd.width * fd.height
	nodeCount := float64(len(graph.Nodes))
	fd.k = math.Sqrt(area / nodeCount)

	// Initialize node positions randomly if not already set
	for i := range graph.Nodes {
		node := &graph.Nodes[i]
		if node.X == 0 && node.Y == 0 {
			fd.nodePositions[node.NodeID] = position{
				x: (fastRand() * fd.width),
				y: (fastRand() * fd.height),
			}
		} else {
			fd.nodePositions[node.NodeID] = position{
				x: node.X,
				y: node.Y,
			}
		}

		// Initialize velocities to zero
		fd.nodeVelocities[node.NodeID] = velocity{vx: 0, vy: 0}
		fd.forces[node.NodeID] = force{fx: 0, fy: 0}
	}

	// Cache edges for quick lookup during force calculation
	fd.edges = make(map[int64]map[int64][]*models.Edge)
	for i := range graph.Edges {
		edge := &graph.Edges[i]
		fromNode := edge.FromNode
		toNode := edge.ToNode

		if fromNode == nil || toNode == nil {
			// Find nodes if references are not set
			for j := range graph.Nodes {
				if graph.Nodes[j].ID == edge.Source {
					fromNode = &graph.Nodes[j]
				}
				if graph.Nodes[j].ID == edge.Target {
					toNode = &graph.Nodes[j]
				}
				if fromNode != nil && toNode != nil {
					break
				}
			}
		}

		if fromNode != nil && toNode != nil {
			fromID := fromNode.NodeID
			toID := toNode.NodeID

			// Initialize maps if needed
			if _, ok := fd.edges[fromID]; !ok {
				fd.edges[fromID] = make(map[int64][]*models.Edge)
			}
			if _, ok := fd.edges[toID]; !ok {
				fd.edges[toID] = make(map[int64][]*models.Edge)
			}

			// Store edge in both directions for quick lookup
			fd.edges[fromID][toID] = append(fd.edges[fromID][toID], edge)
			fd.edges[toID][fromID] = append(fd.edges[toID][fromID], edge)
		}
	}
}

// Step performs one iteration of the layout algorithm
func (fd *ForceDirectedLayout) Step() bool {
	fd.mu.Lock()
	defer fd.mu.Unlock()

	// Check if we've exceeded max iterations or reached stability
	if fd.iterations >= fd.maxIterations || fd.stable {
		return true
	}

	// Reset forces
	for nodeID := range fd.forces {
		fd.forces[nodeID] = force{fx: 0, fy: 0}
	}

	// Apply repulsive forces (nodes repel each other)
	nodeIDs := make([]int64, 0, len(fd.nodePositions))
	for nodeID := range fd.nodePositions {
		nodeIDs = append(nodeIDs, nodeID)
	}

	for i, nodeID1 := range nodeIDs {
		pos1 := fd.nodePositions[nodeID1]

		// Apply gravity to center
		centerX := fd.width / 2
		centerY := fd.height / 2
		dx := centerX - pos1.x
		dy := centerY - pos1.y
		distance := math.Max(0.1, math.Sqrt(dx*dx+dy*dy))

		// Scale gravity by distance from center - stronger pull from far away
		gravityFactor := fd.gravity * (distance / math.Min(fd.width, fd.height))

		fd.forces[nodeID1] = force{
			fx: fd.forces[nodeID1].fx + dx*gravityFactor,
			fy: fd.forces[nodeID1].fy + dy*gravityFactor,
		}

		for j := i + 1; j < len(nodeIDs); j++ {
			nodeID2 := nodeIDs[j]
			pos2 := fd.nodePositions[nodeID2]

			// Vector from node2 to node1
			dx := pos1.x - pos2.x
			dy := pos1.y - pos2.y
			distance := math.Max(0.1, math.Sqrt(dx*dx+dy*dy)) // Prevent division by zero

			// Repulsive force is inversely proportional to distance
			// F = k^2 / distance
			repulsiveForce := (fd.k * fd.k / distance) * fd.repulsionForce / 100.0

			// Apply force in the direction of the vector
			if distance > 0 {
				// Normalize vector
				dx /= distance
				dy /= distance

				// Update forces
				fd.forces[nodeID1] = force{
					fx: fd.forces[nodeID1].fx + dx*repulsiveForce,
					fy: fd.forces[nodeID1].fy + dy*repulsiveForce,
				}
				fd.forces[nodeID2] = force{
					fx: fd.forces[nodeID2].fx - dx*repulsiveForce,
					fy: fd.forces[nodeID2].fy - dy*repulsiveForce,
				}
			}
		}
	}

	// Apply attractive forces (edges pull connected nodes together)
	for nodeID1, connections := range fd.edges {
		pos1 := fd.nodePositions[nodeID1]

		for nodeID2, edges := range connections {
			pos2 := fd.nodePositions[nodeID2]

			// Vector from node1 to node2
			dx := pos2.x - pos1.x
			dy := pos2.y - pos1.y
			distance := math.Max(0.1, math.Sqrt(dx*dx+dy*dy)) // Prevent division by zero

			// Base attractive force is proportional to distance squared
			// F = distance^2 / k
			attractiveForce := distance * distance / fd.k * fd.springConstant

			// Adjust force based on edge weight
			totalWeight := 0.0
			for _, edge := range edges {
				totalWeight += edge.Weight
			}

			// Apply stronger attraction for higher weight edges
			attractiveForce *= (1.0 + totalWeight)

			// Apply force in the direction of the vector
			if distance > 0 {
				// Normalize vector
				dx /= distance
				dy /= distance

				// Update forces
				fd.forces[nodeID1] = force{
					fx: fd.forces[nodeID1].fx + dx*attractiveForce,
					fy: fd.forces[nodeID1].fy + dy*attractiveForce,
				}
				fd.forces[nodeID2] = force{
					fx: fd.forces[nodeID2].fx - dx*attractiveForce,
					fy: fd.forces[nodeID2].fy - dy*attractiveForce,
				}
			}
		}
	}

	// Apply forces with temperature limiting (simulated annealing)
	totalEnergy := 0.0
	for nodeID, f := range fd.forces {
		// Limit force by temperature
		magnitude := math.Sqrt(f.fx*f.fx + f.fy*f.fy)
		if magnitude > 0 {
			// Normalize and scale by temperature
			scale := math.Min(magnitude, fd.temperature) / magnitude
			f.fx *= scale
			f.fy *= scale
		}

		// Update velocity with damping
		v := fd.nodeVelocities[nodeID]
		v.vx = (v.vx + f.fx) * fd.dampingFactor
		v.vy = (v.vy + f.fy) * fd.dampingFactor
		fd.nodeVelocities[nodeID] = v

		// Update position
		pos := fd.nodePositions[nodeID]
		pos.x += v.vx
		pos.y += v.vy

		// Constrain to bounds with padding
		padding := fd.k * 0.5 // Use half the optimal distance as padding
		pos.x = math.Max(padding, math.Min(fd.width-padding, pos.x))
		pos.y = math.Max(padding, math.Min(fd.height-padding, pos.y))
		fd.nodePositions[nodeID] = pos

		// Track energy for convergence detection
		totalEnergy += magnitude
	}

	// Cool temperature (simulated annealing)
	fd.temperature *= 0.95

	// Check if layout is stable
	avgEnergy := totalEnergy / float64(len(fd.forces))
	fd.stable = avgEnergy < fd.energyThreshold

	fd.iterations++
	return fd.stable
}

// Apply updates node positions in the graph
func (fd *ForceDirectedLayout) Apply(graph *models.Graph) {
	fd.mu.Lock()
	defer fd.mu.Unlock()

	// Update node positions in the graph
	for i := range graph.Nodes {
		node := &graph.Nodes[i]
		if pos, ok := fd.nodePositions[node.NodeID]; ok {
			node.X = pos.x
			node.Y = pos.y
		}
	}
}

// edgesFor returns all edges between two nodes
func (fd *ForceDirectedLayout) edgesFor(id1, id2 int64) []*models.Edge {
	if connections, ok := fd.edges[id1]; ok {
		if edges, ok := connections[id2]; ok {
			return edges
		}
	}
	return nil
}

// VoronoiLayout implements a Voronoi-based layout
type VoronoiLayout struct {
	width           float64
	height          float64
	nodePositions   map[int64]position
	centerX         float64
	centerY         float64
	iterations      int
	maxIterations   int
	energyThreshold float64
	mu              sync.Mutex
}

// NewVoronoiLayout creates a new Voronoi-based layout
func NewVoronoiLayout() *VoronoiLayout {
	return &VoronoiLayout{
		width:           800,
		height:          600,
		nodePositions:   make(map[int64]position),
		maxIterations:   300,
		energyThreshold: 0.001,
	}
}

// GetName returns the name of the layout algorithm
func (vl *VoronoiLayout) GetName() string {
	return "Voronoi Layout"
}

// Initialize sets up the Voronoi layout
func (vl *VoronoiLayout) Initialize(graph *models.Graph) {
	vl.mu.Lock()
	defer vl.mu.Unlock()

	vl.width = graph.Width
	vl.height = graph.Height
	vl.maxIterations = graph.MaxIterations
	vl.energyThreshold = graph.StabilizationThreshold
	vl.centerX = vl.width / 2
	vl.centerY = vl.height / 2

	// Arrange nodes in a circle initially
	totalNodes := float64(len(graph.Nodes))
	radius := math.Min(vl.width, vl.height) * 0.4

	for i := range graph.Nodes {
		node := &graph.Nodes[i]
		angle := (2 * math.Pi * float64(i)) / totalNodes

		vl.nodePositions[node.NodeID] = position{
			x: vl.centerX + radius*math.Cos(angle),
			y: vl.centerY + radius*math.Sin(angle),
		}
	}
}

// Step performs one iteration of the Voronoi layout algorithm
func (vl *VoronoiLayout) Step() bool {
	vl.mu.Lock()
	defer vl.mu.Unlock()

	// Check if we've exceeded max iterations
	if vl.iterations >= vl.maxIterations {
		return true
	}

	// For Voronoi layout, we're using Lloyd's algorithm
	// 1. Compute the centroid of each node's Voronoi cell
	// 2. Move the node towards that centroid

	// For simplicity, we'll implement a modified version that pushes nodes away from each other
	// and pulls them toward their connected neighbors

	// Just return true to indicate we're done
	// In a real implementation, we would calculate centroids and move nodes
	vl.iterations++
	return vl.iterations >= vl.maxIterations
}

// Apply updates node positions in the graph
func (vl *VoronoiLayout) Apply(graph *models.Graph) {
	vl.mu.Lock()
	defer vl.mu.Unlock()

	// Update node positions in the graph
	for i := range graph.Nodes {
		node := &graph.Nodes[i]
		if pos, ok := vl.nodePositions[node.NodeID]; ok {
			node.X = pos.x
			node.Y = pos.y
		}
	}
}

// SurrealLayout is a creative layout that applies artistic distortions
type SurrealLayout struct {
	baseLayout        LayoutAlgorithm
	noiseGenerator    opensimplex.Noise
	noiseScale        float64
	timeStep          float64
	distortionAmount  float64
	colorShift        float64
	pulseFactor       float64
	flowFields        bool
	vorticityEnabled  bool
	vorticityStrength float64
	seed              int64
}

// NewSurrealLayout creates a new surreal layout using the specified base layout
func NewSurrealLayout(base LayoutAlgorithm) *SurrealLayout {
	seed := time.Now().UnixNano()
	return &SurrealLayout{
		baseLayout:        base,
		noiseGenerator:    opensimplex.New(seed),
		noiseScale:        0.03,
		timeStep:          0,
		distortionAmount:  20.0,
		colorShift:        0.2,
		pulseFactor:       0.1,
		flowFields:        true,
		vorticityEnabled:  true,
		vorticityStrength: 0.05,
		seed:              seed,
	}
}

// GetName returns the name of the layout algorithm
func (sl *SurrealLayout) GetName() string {
	return "Surreal Layout"
}

// Initialize initializes the surreal layout
func (sl *SurrealLayout) Initialize(graph *models.Graph) {
	sl.baseLayout.Initialize(graph)
}

// Step performs one iteration of the layout algorithm
func (sl *SurrealLayout) Step() bool {
	return sl.baseLayout.Step()
}

// Apply applies the surreal layout to the graph
func (sl *SurrealLayout) Apply(graph *models.Graph) {
	// First, apply the base layout
	sl.baseLayout.Apply(graph)

	// Then, apply surreal distortions and transformations
	for i := range graph.Nodes {
		node := &graph.Nodes[i]

		// Calculate a unique phase for each node based on its ID
		nodePhase := float64(node.NodeID) * 0.1

		// Base distortion using simplex noise
		noise1 := sl.noiseGenerator.Eval3(node.X*sl.noiseScale, node.Y*sl.noiseScale, sl.timeStep)
		noise2 := sl.noiseGenerator.Eval3(node.X*sl.noiseScale+100, node.Y*sl.noiseScale+100, sl.timeStep)

		// Apply distortion with pulsing effect
		pulseAmount := 1.0 + math.Sin(sl.timeStep*2+nodePhase)*sl.pulseFactor
		node.X += noise1 * sl.distortionAmount * pulseAmount
		node.Y += noise2 * sl.distortionAmount * pulseAmount

		// Optional: Apply flow fields for more organic movement
		if sl.flowFields {
			// Calculate flow field vector at node position
			flowX := sl.noiseGenerator.Eval3(node.Y*sl.noiseScale*0.5, node.X*sl.noiseScale*0.3, sl.timeStep*0.2)
			flowY := sl.noiseGenerator.Eval3(node.X*sl.noiseScale*0.5+50, node.Y*sl.noiseScale*0.3+50, sl.timeStep*0.2)

			// Apply flow field force
			flowStrength := 5.0
			node.X += flowX * flowStrength
			node.Y += flowY * flowStrength
		}

		// Optional: Apply vorticity for swirling effects
		if sl.vorticityEnabled {
			// Calculate distance from center
			centerX := graph.Width / 2
			centerY := graph.Height / 2
			dx := node.X - centerX
			dy := node.Y - centerY
			distance := math.Sqrt(dx*dx + dy*dy)

			// Calculate angle for rotation
			angle := sl.vorticityStrength * math.Sin(distance*0.01+sl.timeStep)

			// Rotate point around center
			cosAngle := math.Cos(angle)
			sinAngle := math.Sin(angle)
			node.X = centerX + (dx*cosAngle - dy*sinAngle)
			node.Y = centerY + (dx*sinAngle + dy*cosAngle)
		}

		// Update node size with pulsating effect if desired
		// node.Size *= 1.0 + math.Sin(sl.timeStep*3+nodePhase)*0.2

		// Optionally shift node colors for surreal effect
		// This would require processing the color string, converting to HSL, shifting hue, etc.
	}

	// We can optionally process edges for surreal effects here
	// But we'll skip it for now to avoid unused variable warnings

	// Increment time step for animation
	sl.timeStep += 0.01
}

// Dreamscape layout combines multiple algorithms for a dreamlike effect
type DreamscapeLayout struct {
	forceLayout     *ForceDirectedLayout
	noiseGenerator  opensimplex.Noise
	width           float64
	height          float64
	iterations      int
	maxIterations   int
	timeStep        float64
	groupAttraction float64
	nodeCluster     map[int64]int // Maps node IDs to cluster IDs
	clusterCenters  map[int]position
	mu              sync.Mutex
}

// NewDreamscapeLayout creates a new dreamscape layout
func NewDreamscapeLayout() *DreamscapeLayout {
	return &DreamscapeLayout{
		forceLayout:     NewForceDirectedLayout(),
		noiseGenerator:  opensimplex.New(time.Now().UnixNano()),
		maxIterations:   500,
		groupAttraction: 0.3,
		nodeCluster:     make(map[int64]int),
		clusterCenters:  make(map[int]position),
	}
}

// GetName returns the name of the layout algorithm
func (dl *DreamscapeLayout) GetName() string {
	return "Dreamscape Layout"
}

// Initialize sets up the dreamscape layout
func (dl *DreamscapeLayout) Initialize(graph *models.Graph) {
	dl.mu.Lock()
	defer dl.mu.Unlock()

	dl.width = graph.Width
	dl.height = graph.Height
	dl.maxIterations = graph.MaxIterations

	// Initialize force layout first
	dl.forceLayout.Initialize(graph)

	// Cluster nodes based on their types or properties
	dl.clusterNodes(graph)

	// Initialize cluster centers
	dl.initializeClusterCenters()
}

// Step performs one iteration of the layout algorithm
func (dl *DreamscapeLayout) Step() bool {
	dl.mu.Lock()
	defer dl.mu.Unlock()

	// First step the force layout
	forceStable := dl.forceLayout.Step()

	// Check if we've exceeded max iterations
	if dl.iterations >= dl.maxIterations {
		return true
	}

	dl.iterations++

	// Continue until force layout is stable and we've done enough iterations
	return forceStable && dl.iterations >= dl.maxIterations/2
}

// Apply updates node positions in the graph
func (dl *DreamscapeLayout) Apply(graph *models.Graph) {
	dl.mu.Lock()
	defer dl.mu.Unlock()

	// Apply force layout first
	dl.forceLayout.Apply(graph)

	// Then apply dreamscape transformations
	for i := range graph.Nodes {
		node := &graph.Nodes[i]

		// Apply attraction to cluster center
		if clusterId, ok := dl.nodeCluster[node.NodeID]; ok {
			if center, ok := dl.clusterCenters[clusterId]; ok {
				// Move node towards cluster center
				dx := center.x - node.X
				dy := center.y - node.Y
				node.X += dx * dl.groupAttraction
				node.Y += dy * dl.groupAttraction
			}
		}

		// Apply dreamlike distortions
		noise := dl.noiseGenerator.Eval3(node.X*0.01, node.Y*0.01, dl.timeStep)
		node.X += noise * 5.0
		node.Y += noise * 5.0
	}

	// Increment time step
	dl.timeStep += 0.05
}

// Cluster nodes based on their properties or types
func (dl *DreamscapeLayout) clusterNodes(graph *models.Graph) {
	// Simple clustering based on node types
	clusters := make(map[string]int) // Maps node types to cluster IDs
	nextClusterId := 0

	for i := range graph.Nodes {
		node := &graph.Nodes[i]

		// Get or create cluster ID for this node type
		clusterId, ok := clusters[node.Type]
		if !ok {
			clusterId = nextClusterId
			clusters[node.Type] = clusterId
			nextClusterId++
		}

		// Assign node to cluster
		dl.nodeCluster[node.NodeID] = clusterId
	}
}

// Initialize cluster centers in interesting positions
func (dl *DreamscapeLayout) initializeClusterCenters() {
	clusterCount := len(dl.clusterCenters)
	if clusterCount == 0 {
		return
	}

	// Arrange cluster centers in a circle
	radius := math.Min(dl.width, dl.height) * 0.4
	centerX := dl.width / 2
	centerY := dl.height / 2

	for i := 0; i < clusterCount; i++ {
		angle := (2 * math.Pi * float64(i)) / float64(clusterCount)
		dl.clusterCenters[i] = position{
			x: centerX + radius*math.Cos(angle),
			y: centerY + radius*math.Sin(angle),
		}
	}
}

// Helper functions

// Fast pseudo-random number generator (0-1 range)
var randState uint32 = 1234567890

func fastRand() float64 {
	// Xorshift algorithm
	randState ^= randState << 13
	randState ^= randState >> 17
	randState ^= randState << 5
	return float64(randState) / float64(4294967295) // Normalize to 0-1
}

// GetLayoutAlgorithm returns a layout algorithm by name
func GetLayoutAlgorithm(name string) (LayoutAlgorithm, error) {
	switch name {
	case "force":
		return NewForceDirectedLayout(), nil
	case "surreal":
		return NewSurrealLayout(NewForceDirectedLayout()), nil
	case "voronoi":
		return NewVoronoiLayout(), nil
	case "dreamscape":
		return NewDreamscapeLayout(), nil
	default:
		// Default to force-directed
		return NewForceDirectedLayout(), nil
	}
}

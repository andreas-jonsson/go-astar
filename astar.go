package astar

import "container/heap"

// astar is an A* pathfinding implementation.

// Optional user context.
type Context interface {
	PathNeighbors(node Pather, buf []Pather) []Pather
	PathNeighborCost(node Pather, to Pather) float64
}

// Pather is an interface which allows A* searching on arbitrary objects which
// can represent a weighted graph.
type Pather interface {
	// PathNeighbors returns the direct neighboring nodes of this node which
	// can be pathed to.
	PathNeighbors(ctx Context, buf []Pather) []Pather
	// PathNeighbourCost calculates the exact movement cost to neighbor nodes.
	PathNeighborCost(ctx Context, to Pather) float64
	// PathEstimatedCost is a heuristic method for estimating movement costs
	// between non-adjacent nodes.
	PathEstimatedCost(ctx Context, to Pather) float64
}

// node is a wrapper to store A* data for a Pather node.
type node struct {
	pather Pather
	cost   float64
	rank   float64
	parent *node
	open   bool
	closed bool
	index  int
}

// nodeMap is a collection of nodes keyed by Pather nodes for quick reference.
type nodeMap map[Pather]*node

// get gets the Pather object wrapped in a node, instantiating if required.
func (nm nodeMap) get(p Pather) *node {
	n, ok := nm[p]
	if !ok {
		n = &node{
			pather: p,
		}
		nm[p] = n
	}
	return n
}

type result struct {
	path        []Pather
	distance    float64
	found, done bool
}

// Search is an object representing the current search state.
type Search struct {
	nm       nodeMap
	nq       *priorityQueue
	tmp      []Pather
	fromNode *node
	to       Pather
	ctx      Context
	res      result
}

// NewSearch creates a new search object.
func NewSearch(ctx Context, from, to Pather) *Search {
	s := &Search{}
	s.nm = nodeMap{}
	s.nq = &priorityQueue{}
	s.tmp = make([]Pather, 0, 8)
	heap.Init(s.nq)
	s.fromNode = s.nm.get(from)
	s.fromNode.open = true
	heap.Push(s.nq, s.fromNode)
	s.to = to
	s.ctx = ctx
	return s
}

// PathWithContext calculates a short path and the distance between the two Pather nodes.
// ctx is user optional data.
// If no path is found, found will be false.
func PathWithContext(ctx Context, from, to Pather) (path []Pather, distance float64, found bool) {
	s := NewSearch(ctx, from, to)
	for !s.Step() {
	}
	return s.Result()
}

// Path calculates a short path and the distance between the two Pather nodes.
func Path(from, to Pather) (path []Pather, distance float64, found bool) {
	return PathWithContext(nil, from, to)
}

// Result retrives the final search result.
func (s *Search) Result() (path []Pather, distance float64, found bool) {
	for !s.Step() {
	}
	return s.res.path, s.res.distance, s.res.found
}

// Step advances the search.
//
// Returns true if the search is done.
func (s *Search) Step() bool {
	if s.res.done || s.nq.Len() == 0 {
		// There's no path or we are already done.
		s.res.done = true
		return s.res.done
	}

	current := heap.Pop(s.nq).(*node)
	current.open = false
	current.closed = true

	if current == s.nm.get(s.to) {
		// Found a path to the goal.
		p := []Pather{}
		curr := current
		for curr != nil {
			p = append(p, curr.pather)
			curr = curr.parent
		}

		s.res.path = p
		s.res.distance = current.cost
		s.res.found = true

		return true
	}

	for _, neighbor := range current.pather.PathNeighbors(s.ctx, s.tmp[:0]) {
		cost := current.cost + current.pather.PathNeighborCost(s.ctx, neighbor)
		neighborNode := s.nm.get(neighbor)
		if cost < neighborNode.cost {
			if neighborNode.open {
				heap.Remove(s.nq, neighborNode.index)
			}
			neighborNode.open = false
			neighborNode.closed = false
		}
		if !neighborNode.open && !neighborNode.closed {
			neighborNode.cost = cost
			neighborNode.open = true
			neighborNode.rank = cost + neighbor.PathEstimatedCost(s.ctx, s.to)
			neighborNode.parent = current
			heap.Push(s.nq, neighborNode)
		}
	}

	return false
}

// Context returns the users search context.
func (s *Search) Context() Context {
	return s.ctx
}

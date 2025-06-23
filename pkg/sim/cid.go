package sim

import (
	"fmt"
	"math/rand"
	"time"
)

var preferredAlphas = []rune{'P', 'K', 'N', 'Y', 'T', 'V', 'F', 'R', 'C', 'D', 'E', 'W', 'A'}
var nonPreferredAlphas = []rune{'M', 'X', 'L', 'J', 'U', 'B', 'G', 'Q', 'S', 'H', 'Z'}

// CIDAllocator manages allocation of unique three-character CIDs.
type CIDAllocator struct {
	groups     []map[string]interface{}
	cidGroup   map[string]int
	groupIndex int
	allocated  map[string]struct{}
	rand       *rand.Rand
}

func NewCIDAllocator() *CIDAllocator {
	gs, cm := generateCIDGroups()
	return &CIDAllocator{
		groups:    gs,
		cidGroup:  cm,
		allocated: make(map[string]struct{}),
		rand:      rand.New(rand.NewSource(time.Now().UnixNano())),
	}
}

// Allocate returns the next available CID.
func (c *CIDAllocator) Allocate() (string, error) {
	for c.groupIndex < len(c.groups) {
		g := c.groups[c.groupIndex]
		if len(g) == 0 {
			c.groupIndex++
			continue
		}
		n := c.rand.Intn(len(g))
		var cid string
		i := 0
		for k := range g {
			if i == n {
				cid = k
				break
			}
			i++
		}
		delete(g, cid)
		c.allocated[cid] = struct{}{}
		return cid, nil
	}
	return "", fmt.Errorf("no more CIDs available")
}

// Release frees a CID so it can be reused.
func (c *CIDAllocator) Release(cid string) {
	if _, ok := c.allocated[cid]; !ok {
		return
	}
	delete(c.allocated, cid)
	idx, ok := c.cidGroup[cid]
	if !ok {
		return
	}
	if c.groups[idx] == nil {
		c.groups[idx] = make(map[string]interface{})
	}
	c.groups[idx][cid] = nil
	if idx < c.groupIndex {
		c.groupIndex = idx
	}
}

func generateCIDGroups() ([]map[string]interface{}, map[string]int) {
	digits := []rune{'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'}
	groups := make([]map[string]interface{}, 7)
	cidMap := make(map[string]int)
	add := func(idx int, r1, r2, r3 rune) {
		if groups[idx] == nil {
			groups[idx] = make(map[string]interface{})
		}
		s := string([]rune{r1, r2, r3})
		groups[idx][s] = nil
		cidMap[s] = idx
	}

	for _, a := range digits {
		for _, b := range digits {
			for _, c := range digits {
				add(0, a, b, c)
			}
		}
	}

	for _, a := range digits {
		for _, b := range digits {
			for _, l := range preferredAlphas {
				add(1, a, b, l)
			}
		}
	}

	for _, a := range digits {
		for _, l1 := range preferredAlphas {
			for _, l2 := range preferredAlphas {
				add(2, a, l1, l2)
			}
		}
	}

	for _, a := range digits {
		for _, l := range preferredAlphas {
			for _, b := range digits {
				add(3, a, l, b)
			}
		}
	}

	for _, a := range digits {
		for _, b := range digits {
			for _, l := range nonPreferredAlphas {
				add(4, a, b, l)
			}
		}
	}

	for _, d := range digits {
		for _, a1 := range nonPreferredAlphas {
			for _, a2 := range nonPreferredAlphas {
				add(5, d, a1, a2)
			}
			for _, a2 := range preferredAlphas {
				add(5, d, a1, a2)
			}
		}
		for _, a1 := range preferredAlphas {
			for _, a2 := range nonPreferredAlphas {
				add(5, d, a1, a2)
			}
		}
	}

	for _, a := range digits {
		for _, l := range nonPreferredAlphas {
			for _, b := range digits {
				add(6, a, l, b)
			}
		}
	}

	return groups, cidMap
}

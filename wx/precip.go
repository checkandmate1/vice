package wx

import (
	"fmt"
	"image"
	"image/draw"
	"image/png"
	"io"
	"net/http"
	"net/url"
	"sort"
	"sync"

	"github.com/mmp/vice/math"
	"github.com/mmp/vice/util"

	"github.com/klauspost/compress/zstd"
	"github.com/vmihailenco/msgpack/v5"
)

// Precip is the object type that is stored in GCS after wx ingest for precipitation.
type Precip struct {
	DBZ        []byte
	Resolution int
	Latitude   float32
	Longitude  float32
}

func DecodePrecip(r io.Reader) (*Precip, error) {
	zr, err := zstd.NewReader(r, zstd.WithDecoderConcurrency(0))
	if err != nil {
		return nil, err
	}
	defer zr.Close()

	var precip Precip
	if err := msgpack.NewDecoder(zr).Decode(&precip); err != nil {
		return nil, err
	}

	precip.DBZ = util.DeltaDecode(precip.DBZ)

	return &precip, nil
}

// lat-long bounds
func (p Precip) BoundsLL() math.Extent2D {
	centerLL := math.Point2LL{p.Longitude, p.Latitude}

	// Resolution is in pixels, and we have 0.5nm per pixel (2 samples per nm)
	widthNM := float32(p.Resolution) / 2
	return math.BoundLatLongCircle(centerLL, widthNM/2 /* radius */)
}

func FetchRadarImage(center math.Point2LL, radius float32, resolution int) (image.Image, math.Extent2D, error) {
	// The weather radar image comes via a WMS GetMap request from the NOAA.
	//
	// Relevant background:
	// https://enterprise.arcgis.com/en/server/10.3/publish-services/windows/communicating-with-a-wms-service-in-a-web-browser.htm
	// http://schemas.opengis.net/wms/1.3.0/capabilities_1_3_0.xsd
	// NOAA weather: https://opengeo.ncep.noaa.gov/geoserver/www/index.html
	// https://opengeo.ncep.noaa.gov/geoserver/conus/conus_bref_qcd/ows?service=wms&version=1.3.0&request=GetCapabilities
	params := url.Values{}
	params.Add("SERVICE", "WMS")
	params.Add("REQUEST", "GetMap")
	params.Add("FORMAT", "image/png")
	params.Add("WIDTH", fmt.Sprintf("%d", resolution))
	params.Add("HEIGHT", fmt.Sprintf("%d", resolution))
	params.Add("LAYERS", "conus_bref_qcd")
	bbox := math.BoundLatLongCircle(center, radius)
	params.Add("BBOX", fmt.Sprintf("%f,%f,%f,%f", bbox.P0[0], bbox.P0[1], bbox.P1[0], bbox.P1[1]))

	url := "https://opengeo.ncep.noaa.gov/geoserver/conus/conus_bref_qcd/ows?" + params.Encode()

	resp, err := http.Get(url)
	if err != nil {
		return nil, math.Extent2D{}, err
	}
	defer resp.Body.Close()

	img, err := png.Decode(resp.Body)
	return img, bbox, err
}

// NWS standard base reflectivity dBZ→RGB color table. These are the colors
// the NOAA GeoServer actually renders for conus_bref_qcd.
type nwsColorEntry struct {
	dbz     float32
	r, g, b byte
}

var nwsReflectivityColors = []nwsColorEntry{
	{-25, 0, 0, 0},
	{5, 0, 236, 236},
	{10, 1, 160, 246},
	{15, 0, 0, 246},
	{20, 0, 255, 0},
	{25, 0, 200, 0},
	{30, 0, 144, 0},
	{35, 255, 255, 0},
	{40, 231, 192, 0},
	{45, 255, 144, 0},
	{50, 255, 0, 0},
	{55, 214, 0, 0},
	{60, 192, 0, 0},
	{65, 255, 0, 255},
	{70, 153, 85, 201},
	{75, 255, 255, 255},
}

type kdNode struct {
	rgb [3]byte
	dbz float32
	c   [2]*kdNode
}

func makeRadarKdTree() *kdNode {
	type rgbRefl struct {
		rgb [3]byte
		dbz float32
	}

	// Generate ~500 entries by linearly interpolating between adjacent NWS
	// color pairs, so that the k-d tree has fine-grained coverage of the
	// color space the NOAA GeoServer actually produces.
	const stepsPerSegment = 32
	var r []rgbRefl
	for i := range len(nwsReflectivityColors) - 1 {
		c0 := nwsReflectivityColors[i]
		c1 := nwsReflectivityColors[i+1]
		for s := range stepsPerSegment {
			t := float32(s) / float32(stepsPerSegment)
			r = append(r, rgbRefl{
				rgb: [3]byte{
					byte(math.Lerp(t, float32(c0.r), float32(c1.r)) + 0.5),
					byte(math.Lerp(t, float32(c0.g), float32(c1.g)) + 0.5),
					byte(math.Lerp(t, float32(c0.b), float32(c1.b)) + 0.5),
				},
				dbz: math.Lerp(t, c0.dbz, c1.dbz),
			})
		}
	}
	// Add the final color entry.
	last := nwsReflectivityColors[len(nwsReflectivityColors)-1]
	r = append(r, rgbRefl{rgb: [3]byte{last.r, last.g, last.b}, dbz: last.dbz})

	// Build a kd-tree over the RGB points in the color map.
	var buildTree func(r []rgbRefl, depth int) *kdNode
	buildTree = func(r []rgbRefl, depth int) *kdNode {
		if len(r) == 0 {
			return nil
		}
		if len(r) == 1 {
			return &kdNode{rgb: r[0].rgb, dbz: r[0].dbz}
		}

		// The split dimension cycles through RGB with tree depth.
		dim := depth % 3

		// Sort the points in the current dimension (we actually just need
		// to partition around the midpoint, but...)
		sort.Slice(r, func(i, j int) bool {
			return r[i].rgb[dim] < r[j].rgb[dim]
		})

		// Split in the middle and recurse
		mid := len(r) / 2
		return &kdNode{
			rgb: r[mid].rgb,
			dbz: r[mid].dbz,
			c:   [2]*kdNode{buildTree(r[:mid], depth+1), buildTree(r[mid+1:], depth+1)},
		}
	}

	return buildTree(r, 0)
}

// Returns estimated dBZ (https://en.wikipedia.org/wiki/DBZ_(meteorology)) for
// an RGB by going backwards from the color ramp.
func estimateDBZ(root *kdNode, rgb [3]byte) float32 {
	// All white -> ~nil
	if rgb[0] == 255 && rgb[1] == 255 && rgb[2] == 255 {
		return -100
	}

	// Returns the distnace between the specified RGB and the RGB passed to
	// estimateDBZ.
	dist := func(o []byte) float32 {
		d2 := math.Sqr(int(o[0])-int(rgb[0])) + math.Sqr(int(o[1])-int(rgb[1])) + math.Sqr(int(o[2])-int(rgb[2]))
		return math.Sqrt(float32(d2))
	}

	var searchTree func(n *kdNode, closestNode *kdNode, closestDist float32, depth int) (*kdNode, float32)
	searchTree = func(n *kdNode, closestNode *kdNode, closestDist float32, depth int) (*kdNode, float32) {
		if n == nil {
			return closestNode, closestDist
		}

		// Check the current node
		d := dist(n.rgb[:])
		if d < closestDist {
			closestDist = d
			closestNode = n
		}

		// Split dimension as in buildTree above
		dim := depth % 3

		// Initially traverse the tree based on which side of the split
		// plane the lookup point is on.
		var first, second *kdNode
		if rgb[dim] < n.rgb[dim] {
			first, second = n.c[0], n.c[1]
		} else {
			first, second = n.c[1], n.c[0]
		}

		closestNode, closestDist = searchTree(first, closestNode, closestDist, depth+1)

		// If the distance to the split plane is less than the distance to
		// the closest point found so far, we need to check the other side
		// of the split.
		if float32(math.Abs(int(rgb[dim])-int(n.rgb[dim]))) < closestDist {
			closestNode, closestDist = searchTree(second, closestNode, closestDist, depth+1)
		}

		return closestNode, closestDist
	}

	n, _ := searchTree(root, nil, 100000, 0)
	return n.dbz
}

// Allow concurrent calls to RadarImageToDBZ
var getRadarKdTree = sync.OnceValue(func() *kdNode { return makeRadarKdTree() })

func RadarImageToDBZ(img image.Image) []byte {
	// Convert the Image returned by png.Decode to a simple 8-bit RGBA image.
	rgba := image.NewRGBA(img.Bounds())
	draw.Draw(rgba, img.Bounds(), img, image.Point{}, draw.Over)

	root := getRadarKdTree()

	// Determine the dBZ for each pixel.
	ny, nx := img.Bounds().Dy(), img.Bounds().Dx()
	dbzImage := make([]byte, nx*ny)
	for y := range ny {
		for x := range nx {
			px := rgba.RGBAAt(x, y)
			dbz := estimateDBZ(root, [3]byte{px.R, px.G, px.B})

			dbzImage[x+y*nx] = byte(max(0, min(255, dbz)))
		}
	}

	return dbzImage
}

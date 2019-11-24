// Datastructures.hh

#ifndef DATASTRUCTURES_HH
#define DATASTRUCTURES_HH

#include <string>
#include <vector>
#include <utility>
#include <limits>
#include <unordered_map>
#include <algorithm>
#include <memory>
#include <map>
#include <stack>
#include <queue>

// Type for beacon IDs
using BeaconID = std::string;

// Return value for cases where required beacon was not found
BeaconID const NO_ID = "----------";

// Return value for cases where integer values were not found
int const NO_VALUE = std::numeric_limits<int>::min();

// Return value for cases where name values were not found
std::string const NO_NAME = "-- unknown --";

// Type for a coordinate (x, y)
struct Coord
{
    int x = NO_VALUE;
    int y = NO_VALUE;
};

// Example: Defining == and hash function for Coord so that it can be used
// as key for std::unordered_map/set, if needed
inline bool operator==(Coord c1, Coord c2) { return c1.x == c2.x && c1.y == c2.y; }
inline bool operator!=(Coord c1, Coord c2) { return !(c1==c2); } // Not strictly necessary

struct CoordHash
{
    std::size_t operator()(Coord xy) const
    {
        auto hasher = std::hash<int>();
        auto xhash = hasher(xy.x);
        auto yhash = hasher(xy.y);
        // Combine hash values (magic!)
        return xhash ^ (yhash + 0x9e3779b9 + (xhash << 6) + (xhash >> 2));
    }
};

// Example: Defining < for Coord so that it can be used
// as key for std::map/set
inline bool operator<(Coord c1, Coord c2)
{
    if (c1.y < c2.y) { return true; }
    else if (c2.y < c1.y) { return false; }
    else { return c1.x < c2.x; }
}

// Return value for cases where coordinates were not found
Coord const NO_COORD = {NO_VALUE, NO_VALUE};

// Type for color (RGB)
struct Color
{
    int r = NO_VALUE;
    int g = NO_VALUE;
    int b = NO_VALUE;
};

// Equality and non-equality comparisons for Colors
inline bool operator==(Color c1, Color c2) { return c1.r == c2.r && c1.g == c2.g && c1.b == c2.b; }
inline bool operator!=(Color c1, Color c2) { return !(c1==c2); }

// Return value for cases where color was not found
Color const NO_COLOR = {NO_VALUE, NO_VALUE, NO_VALUE};

// Type for light transmission cost (used only in the second assignment)
using Cost = int;

// Return value for cases where cost is unknown
Cost const NO_COST = NO_VALUE;

// The datastructures below are made by me

// The structure for beacon objects
struct Beacon
{
    BeaconID id = NO_ID;
    BeaconID target = NO_ID;
    std::vector<BeaconID> sources;
    bool isSourcesSorted = false;
    std::string name = NO_NAME;
    Coord xy = NO_COORD;
    Color color = NO_COLOR;
};

// Return value for cases where a vector is needed and id was not found
std::vector<BeaconID> const NO_FIND {NO_ID};


struct Fibre;

struct Xpoint {
    Coord coord;
    std::string color;
    Cost d;
    std::shared_ptr<Xpoint> pi;
    // A unordered_map that contains all fibres leaving from the Xpoint
    // the key is the "destination" Xpoint
    std::unordered_map<std::shared_ptr<Xpoint>, std::shared_ptr<Fibre>> fibres;
};

struct Fibre {
    std::pair<Coord, Coord> headCoords;
    Cost cost;
};


// This is the class I implemented

class Datastructures
{
public:
    Datastructures();
    ~Datastructures();

    // Estimate of performance: O(1)
    // Short rationale for estimate: size function has a time of O(1)
    int beacon_count();

    // Estimate of performance: O(n)
    // Short rationale for estimate: clear function time for maps
    void clear_beacons();

    // Estimate of performance: O(n)
    // Short rationale for estimate: for loop going trough the elements
    std::vector<BeaconID> all_beacons();

    // Estimate of performance: O(log(n))
    // Short rationale for estimate: multimap insertion
    bool add_beacon(BeaconID id, std::string const& name, Coord xy,
                    Color color);

    // Estimate of performance: O(1)
    // Short rationale for estimate: find and []-operator are constant
    std::string get_name(BeaconID id);

    // Estimate of performance: O(1)
    // Short rationale for estimate: find and []-operator are constant
    Coord get_coordinates(BeaconID id);

    // Estimate of performance: O(1)
    // Short rationale for estimate: find and []-operator are constant
    Color get_color(BeaconID id);

    // Estimate of performance: O(n)
    // Short rationale for estimate: for looping trough the elements
    std::vector<BeaconID> beacons_alphabetically();

    // Estimate of performance: O(n)
    // Short rationale for estimate: for looping trough the elements
    std::vector<BeaconID> beacons_brightness_increasing();

    // Estimate of performance: O(1)
    // Short rationale for estimate: uses iterators to find the first value
    BeaconID min_brightness();

    // Estimate of performance: O(1)
    // Short rationale for estimate: uses iterators to find the last value
    BeaconID max_brightness();

    // Estimate of performance: O(nlog(n))
    // Short rationale for estimate: std::sort algorithm
    std::vector<BeaconID> find_beacons(std::string const& name);

    // Estimate of performance: O(n)
    // Short rationale for estimate: for loop where one erase is called
    bool change_beacon_name(BeaconID id, std::string const& newname);

    // Estimate of performance: O(n)
    // Short rationale for estimate: for loop where one erase is called
    bool change_beacon_color(BeaconID id, Color newcolor);

    // Estimate of performance: O(1)
    // Short rationale for estimate: find and []-operator are constant
    bool add_lightbeam(BeaconID sourceid, BeaconID targetid);

    // Estimate of performance: O(nlog(n)), if the lightsources are sorted O(1)
    // Short rationale for estimate: std::sort algorithm, if the lightsources
    // are sorted no sort algorithm is called
    std::vector<BeaconID> get_lightsources(BeaconID id);

    // Estimate of performance: O(n)
    // Short rationale for estimate: while looping trough the elements
    std::vector<BeaconID> path_outbeam(BeaconID id);

    // Estimate of performance: O(n + m), where m is size of the sources vector
    // Short rationale for estimate: for loop where one erase is called
    // m: std::find used to find an element from a vector
    bool remove_beacon(BeaconID id);

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<BeaconID> path_inbeam_longest(BeaconID id);

    // Estimate of performance: O(n)
    // Short rationale for estimate: uses the function total_color_recursion
    Color total_color(BeaconID id);

    // Estimate of performance: if the vector is sorted O(1) if not O(nlog(n))
    // Short rationale for estimate: uses std::sort algorithm
    std::vector<Coord> all_xpoints();

    // Estimate of performance: O(1)
    // Short rationale for estimate: unordered_map.insert and vector.push_back
    // and unordered_map.find are constant
    bool add_fibre(Coord xpoint1, Coord xpoint2, Cost cost);

    // Estimate of performance: O(nlog(n))
    // Short rationale for estimate: uses std::sort algorithm
    std::vector<std::pair<Coord, Cost>> get_fibres_from(Coord xpoint);

    // Estimate of performance: O(n)
    // Short rationale for estimate: for loops trough a map
    std::vector<std::pair<Coord, Coord>> all_fibres();

    // Estimate of performance: O(1)
    // Short rationale for estimate: unordered_map.erase and find are constant
    bool remove_fibre(Coord xpoint1, Coord xpoint2);

    // Estimate of performance: O(n)
    // Short rationale for estimate: clear functions for vector and
    // unordered_map are linear in the size of the container
    void clear_fibres();

    // Estimate of performance: O(V+E log(V))
    // Short rationale for estimate: uses dijkstra algorithm to find the path
    std::vector<std::pair<Coord, Cost>> route_any(Coord fromxpoint, Coord toxpoint);

    // Estimate of performance: O(V+E)
    // Short rationale for estimate: uses bfs algorithm to find the path
    std::vector<std::pair<Coord, Cost>> route_least_xpoints(Coord fromxpoint, Coord toxpoint);

    // Estimate of performance: O(V+E log(V))
    // Short rationale for estimate: uses dijkstra algorithm to find the path
    std::vector<std::pair<Coord, Cost>> route_fastest(Coord fromxpoint, Coord toxpoint);

    // Estimate of performance: O(V+E)
    // Short rationale for estimate: uses dfs algorithm to find the cycle
    std::vector<Coord> route_fibre_cycle(Coord startxpoint);

    // Estimate of performance:
    // Short rationale for estimate:
    Cost trim_fibre_network();

private:
    // Estimate of performance: O(n) where n is the amount of nodes in the
    // tree. The root of the tree is the first id
    // Short rationale for estimate: Looping trough the tree with a recursion
    Color total_color_recursion(const BeaconID &id);

    std::unordered_map<BeaconID, Beacon> beacons_;
    std::multimap<std::string, BeaconID> nameSorted_;
    std::multimap<int, BeaconID> brightSorted_;

    // Containers and help variable for sorting
    std::unordered_map<Coord, std::shared_ptr<Xpoint>, CoordHash> xpoints_;
    std::map<std::pair<Coord, Coord>, std::shared_ptr<Fibre>> fibres_;
    std::vector<Coord> allXpoints_;
    bool isXpointsSorted_ = false;

    //  support functions and needed algorithms

    // Resets the Xpoints to default settings
    void reset_xpoints();

    // A needed function for dijkstra algorithm
    void relax(std::shared_ptr<Xpoint> u, std::shared_ptr<Xpoint> v, Cost w);

    // Dijkstra algorithm with a performance of O(V+E log(V))
    bool dijkstra(std::shared_ptr<Xpoint> start,
                  std::shared_ptr<Xpoint> end = nullptr);

    // Depth first search for route_fibre_cycle method with a perf of O(V+E)
    std::shared_ptr<Xpoint> dfs(std::shared_ptr<Xpoint> start);

    // Breadth first search for route_least_xpoints method with a perf of O(V+E)
    bool bfs(std::shared_ptr<Xpoint> start,
             std::shared_ptr<Xpoint> end = nullptr);

    // Makes a coordinate pair where the first coordinate is smaller
    std::pair<Coord, Coord> make_pair(Coord xpoint1, Coord xpoint2);

    // Compare operator for Xpoints
    struct XpointCmp
    {
        bool operator()(std::shared_ptr<Xpoint> x1, std::shared_ptr<Xpoint> x2);
    };

};

#endif // DATASTRUCTURES_HH

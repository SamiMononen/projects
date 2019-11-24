// Datastructures.cc

#include "datastructures.hh"

#include <random>

#include <cmath>

std::minstd_rand rand_engine; // Reasonably quick pseudo-random generator

template <typename Type>
Type random_in_range(Type start, Type end)
{
    auto range = end-start;
    ++range;

    auto num = std::uniform_int_distribution<unsigned long int>(0, range-1)(rand_engine);

    return static_cast<Type>(start+num);
}

// Everything below this point is my own code

Datastructures::Datastructures()
{
}

Datastructures::~Datastructures()
{
}

int Datastructures::beacon_count()
{
    auto size = beacons_.size();
    int convertdata = static_cast<int>(size);
    return convertdata;
}

void Datastructures::clear_beacons()
{
    beacons_.clear();
    nameSorted_.clear();
    brightSorted_.clear();
}

std::vector<BeaconID> Datastructures::all_beacons()
{
    std::vector<BeaconID> all_id;
    for(auto beacon : beacons_){
        all_id.push_back(beacon.first);
    }
    return all_id;
}

bool Datastructures::add_beacon(BeaconID id, const std::string& name,
                                Coord xy, Color color)
{
    if(beacons_.find(id) != beacons_.end()){
        return false;
    }
    Beacon beacon;
    beacon.id = id;
    beacon.name = name;
    beacon.xy = xy;
    beacon.color = color;
    int brightness = 3*beacon.color.r+6*beacon.color.g+beacon.color.b;

    brightSorted_.insert({brightness, id});
    nameSorted_.insert({name, id});
    beacons_.insert({id, beacon});
    return true;
}

std::string Datastructures::get_name(BeaconID id)
{
    if(beacons_.find(id) == beacons_.end()){
        return NO_NAME;
    }
    return beacons_[id].name;
}

Coord Datastructures::get_coordinates(BeaconID id)
{
    if(beacons_.find(id) == beacons_.end()){
        return NO_COORD;;
    }
    return beacons_[id].xy;
}

Color Datastructures::get_color(BeaconID id)
{
    if(beacons_.find(id) == beacons_.end()){
        return NO_COLOR;
    }
    return beacons_[id].color;
}

std::vector<BeaconID> Datastructures::beacons_alphabetically()
{
    std::vector<BeaconID> alphapetic_beacons;

    for(auto beacon : nameSorted_){
        alphapetic_beacons.push_back(beacon.second);
    }
    return alphapetic_beacons;
}

std::vector<BeaconID> Datastructures::beacons_brightness_increasing()
{
    std::vector<BeaconID> brightness_beacons;

    for(auto beacon : brightSorted_){
        brightness_beacons.push_back(beacon.second);
    }
    return brightness_beacons;
}

BeaconID Datastructures::min_brightness()
{
    if(beacons_.size() == 0){
        return NO_ID;
    }
    return brightSorted_.begin()->second;
}

BeaconID Datastructures::max_brightness()
{
    if(beacons_.size() == 0){
        return NO_ID;
    }
    return brightSorted_.rbegin()->second;
}

std::vector<BeaconID> Datastructures::find_beacons(std::string const& name)
{
    std::vector<BeaconID> sameName;
    for(auto beacon : nameSorted_){
        if(beacon.first == name){
            sameName.push_back(beacon.second);
        }
    }
    std::sort(sameName.begin(), sameName.end());
    return sameName;
}

bool Datastructures::change_beacon_name(BeaconID id,
                                        const std::string& newname)
{
    if(beacons_.find(id) == beacons_.end()){
        return false;
    }

    // Finds the beacon to be changed and erases it
    std::multimap<std::string, BeaconID>::iterator it;
    for(it = nameSorted_.begin(); it != nameSorted_.end(); it++){
        if(it->second == id){
            it = nameSorted_.erase(it);
            break;
        }
    }
    // Inserting the new values
    nameSorted_.insert({newname, id});
    beacons_[id].name = newname;
    return true;
}

bool Datastructures::change_beacon_color(BeaconID id, Color newcolor)
{
    if(beacons_.find(id) == beacons_.end()){
        return false;
    }
    int newBrightness = 3*newcolor.r+6*newcolor.g+newcolor.b;

    // Finds the beacon to be changed and erases it
    std::multimap<int, BeaconID>::iterator it;
    for(it = brightSorted_.begin(); it != brightSorted_.end(); it++){
        if(it->second == id){
            it = brightSorted_.erase(it);
            break;
        }
    }
    // Inserting the new values
    brightSorted_.insert({newBrightness, id});
    beacons_[id].color = newcolor;
    return true;
}

bool Datastructures::add_lightbeam(BeaconID sourceid, BeaconID targetid)
{
    if(beacons_.find(sourceid) == beacons_.end() ||
            beacons_.find(targetid) == beacons_.end()){
        return false;
    }
    else if(beacons_[sourceid].target != NO_ID){
        return false;
    }
    beacons_[sourceid].target = targetid;
    beacons_[targetid].sources.push_back(sourceid);
    beacons_[targetid].isSourcesSorted = false;
    return true;
}

std::vector<BeaconID> Datastructures::get_lightsources(BeaconID id)
{
    if(beacons_.find(id) == beacons_.end()){
        return NO_FIND;
    }
    // Checking if the lightsource is sorted to save time
    if(beacons_[id].isSourcesSorted == false){
        std::sort(beacons_[id].sources.begin(), beacons_[id].sources.end());
        beacons_[id].isSourcesSorted = true;
    }
    return beacons_[id].sources;
}

std::vector<BeaconID> Datastructures::path_outbeam(BeaconID id)
{
    BeaconID currentID = id;
    std::vector<BeaconID> pathOutbeam;
    if(beacons_.find(id) == beacons_.end()){
        return NO_FIND;
    }
    pathOutbeam.push_back(id);

    while(beacons_.find(currentID)->second.target != NO_ID){
        pathOutbeam.push_back(beacons_.find(currentID)->second.target);
        currentID = beacons_.find(currentID)->second.target;
    }
    return pathOutbeam;
}

bool Datastructures::remove_beacon(BeaconID id)
{
    if(beacons_.find(id) == beacons_.end()){
        return false;
    }

    // Changes the target to be NO_ID for all the beacons sources
    for(auto beacon : beacons_[id].sources){
        beacons_.find(beacon)->second.target = NO_ID;
    }

    // Removes the beacon being removed from being a source
    auto itT = beacons_.find(beacons_[id].target);
    if(itT != beacons_.end()){
        auto itS = std::find(itT->second.sources.begin(),
                             itT->second.sources.end(), id);
        itT->second.sources.erase(itS);
    }

    beacons_.erase(id);

    // Removes the beacon from the sorted maps
    std::multimap<int, BeaconID>::iterator it;
    for(it = brightSorted_.begin(); it != brightSorted_.end(); it++){
        if(it->second == id){
            brightSorted_.erase(it);
            break;
        }
    }
    std::multimap<std::string, BeaconID>::iterator itN;
    for(itN = nameSorted_.begin(); itN != nameSorted_.end(); itN++){
        if(itN->second == id){
            nameSorted_.erase(itN);
            break;
        }
    }
    return true;
}

std::vector<BeaconID> Datastructures::path_inbeam_longest(BeaconID /*id*/)
{
    // Replace this with your implementation
    return {{NO_ID}};
}

Color Datastructures::total_color_recursion(const BeaconID &id)
{
    Beacon &beacon = beacons_.find(id)->second;
    Color totalColor = beacon.color;

    // The ending condition for the recourse
    if(beacon.sources.empty()){
        return beacon.color;
    }

    int denominator = static_cast<int>(beacon.sources.size()) + 1;

    // The loop goes trough all the sources and their sources adding their
    // colors to the totalColor
    for(auto sourceB : beacon.sources){
        Color sumColor = total_color_recursion(sourceB);

        totalColor.r += sumColor.r;
        totalColor.g += sumColor.g;
        totalColor.b += sumColor.b;
    }
    totalColor.r /= denominator;
    totalColor.g /= denominator;
    totalColor.b /= denominator;
    return totalColor;
}



Color Datastructures::total_color(BeaconID id)
{
    if(beacons_.find(id) == beacons_.end()){
            return NO_COLOR;
        }
    return total_color_recursion(id);
}


std::vector<Coord> Datastructures::all_xpoints()
{
    // If xpoints are not sorted sort them again
    if(isXpointsSorted_ == false){
        allXpoints_.clear();
        for(auto x : xpoints_){
            allXpoints_.push_back(x.first);
        }
        std::sort(allXpoints_.begin(), allXpoints_.end());
        isXpointsSorted_ = true;
    }
    return allXpoints_;
}

bool Datastructures::add_fibre(Coord xpoint1, Coord xpoint2, Cost cost)
{
    if(xpoint1 == xpoint2){
        return false;
    }

    // Making a pair of coordinates with a help method that makes sure the
    // first coord value is always lower
    std::pair<Coord, Coord> headCoords = make_pair(xpoint1, xpoint2);

    if(fibres_.find(headCoords) != fibres_.end()){
        return false;
    }

    std::unordered_map<std::shared_ptr<Xpoint>, std::shared_ptr<Fibre>> fibres;

    // Checking if the xpoints exist to avoid duplicates
    if(xpoints_.find(xpoint1) == xpoints_.end()){
        auto xPoint1 = std::make_shared<Xpoint>(Xpoint{xpoint1, "white", -1, nullptr, fibres});
        xpoints_.insert({xpoint1, xPoint1});
        allXpoints_.push_back(xpoint1);
        isXpointsSorted_ = false;
    }
    if(xpoints_.find(xpoint2) == xpoints_.end()){
        auto xPoint2 = std::make_shared<Xpoint>(Xpoint{xpoint2, "white", -1, nullptr, fibres});
        xpoints_.insert({xpoint2, xPoint2});
        allXpoints_.push_back(xpoint2);
        isXpointsSorted_ = false;
    }

    auto point1 = xpoints_.find(xpoint1)->second;
    auto point2 = xpoints_.find(xpoint2)->second;

    auto fibre = std::make_shared<Fibre>(Fibre{headCoords, cost});
    // Inserting the fibre to the fibres map of the Xpoints
    point1->fibres.insert({point2, fibre});
    point2->fibres.insert({point1, fibre});
    fibres_.insert({headCoords, fibre});

    return true;
}

std::vector<std::pair<Coord, Cost> > Datastructures::get_fibres_from(Coord xpoint)
{
    std::vector<std::pair<Coord, Cost>> fibreInfoVector;

    auto it = xpoints_.find(xpoint);

    if(it == xpoints_.end()){
        return fibreInfoVector;
    }

    // Getting the connections from unordered_map fibre of the given Xpoint
    for(auto fibre : it->second->fibres){
        Coord otherSide = fibre.first->coord;
        Cost fibreCost = fibre.second->cost;
        std::pair<Coord, Cost> fibreInfo = std::make_pair(otherSide,
                                                          fibreCost);
        fibreInfoVector.push_back(fibreInfo);
    }
    std::sort(fibreInfoVector.begin(), fibreInfoVector.end());
    return fibreInfoVector;
}

std::vector<std::pair<Coord, Coord> > Datastructures::all_fibres()
{
    std::vector<std::pair<Coord, Coord>> allFibres;
    for(auto fibre : fibres_){
        allFibres.push_back(fibre.first);
    }
    return allFibres;
}

bool Datastructures::remove_fibre(Coord xpoint1, Coord xpoint2)
{
    std::pair<Coord, Coord> headCoords = make_pair(xpoint1, xpoint2);

    auto it = fibres_.find(headCoords);

    if(it == fibres_.end()){
         return false;
    }

    auto xpoint1It = xpoints_.find(xpoint1);
    auto xpoint2It = xpoints_.find(xpoint2);

    xpoint1It->second->fibres.erase(xpoint2It->second);
    xpoint2It->second->fibres.erase(xpoint1It->second);

    // Check if one of the xpoints is isolated and removes it if it is
    if(xpoint1It->second->fibres.empty()){
        xpoints_.erase(xpoint1It);
        isXpointsSorted_ = false;
    }
    if(xpoint2It->second->fibres.empty()){
        xpoints_.erase(xpoint2It);
        isXpointsSorted_ = false;
    }

    fibres_.erase(it);
    return true;
}

void Datastructures::clear_fibres()
{
    for(auto xpoint : xpoints_){
        xpoint.second->fibres.clear();
    }
    allXpoints_.clear();
    fibres_.clear();
    xpoints_.clear();
}

std::vector<std::pair<Coord, Cost>> Datastructures::route_any(Coord fromxpoint,
                                                               Coord toxpoint)
{
    auto fromIt = xpoints_.find(fromxpoint);
    auto toIt = xpoints_.find(toxpoint);

    if(fromIt == xpoints_.end() or
            toIt == xpoints_.end()){
        return {};
    }

    // Dijkstra algorithm
    bool route = dijkstra(fromIt->second, toIt->second);

    // The route wasnt found
    if(route == false){
        return {};
    }

    std::vector<std::pair<Coord, Cost>> path;
    std::shared_ptr<Xpoint> u = toIt->second;

    while(u != nullptr){
        Coord coord = u->coord;
        Cost w = u->d;
        std::pair<Coord,Cost> pathPair = std::make_pair(coord, w);
        path.push_back(pathPair);
        u = u->pi;
    }
    // Reversing the vector to make it start from the right node and price
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<std::pair<Coord, Cost>> Datastructures::route_least_xpoints(Coord fromxpoint, Coord toxpoint)
{
    auto fromIt = xpoints_.find(fromxpoint);
    auto toIt = xpoints_.find(toxpoint);

    if(fromIt == xpoints_.end() or
            toIt == xpoints_.end()){
        return {};
    }

    bool route = bfs(fromIt->second, toIt->second);

    if(route == false){
        return {};
    }
    std::vector<std::pair<Coord, Cost>> path;
    std::shared_ptr<Xpoint> u = toIt->second;

    while(u != nullptr){
        Coord coord = u->coord;
        Cost w = u->d;
        std::pair<Coord,Cost> pathPair = std::make_pair(coord, w);
        path.push_back(pathPair);
        u = u->pi;
    }
    // Reversing the vector to make it start from the right node and price
    std::reverse(path.begin(), path.end());
    return path;

}

std::vector<std::pair<Coord, Cost>> Datastructures::route_fastest(Coord fromxpoint,
                                                                  Coord toxpoint)
{
    auto fromIt = xpoints_.find(fromxpoint);
    auto toIt = xpoints_.find(toxpoint);

    if(fromIt == xpoints_.end() or
            toIt == xpoints_.end()){
        return {};
    }

    // Dijkstra algorithm
    bool route = dijkstra(fromIt->second, toIt->second);

    // The route wasnt found
    if(route == false){
        return {};
    }

    std::vector<std::pair<Coord, Cost>> path;
    std::shared_ptr<Xpoint> u = toIt->second;

    while(u != nullptr){
        Coord coord = u->coord;
        Cost w = u->d;
        std::pair<Coord,Cost> pathPair = std::make_pair(coord, w);
        path.push_back(pathPair);
        u = u->pi;
    }
    // Reversing the vector to make it start from the right node and price
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Coord> Datastructures::route_fibre_cycle(Coord startxpoint)
{
    auto it = xpoints_.find(startxpoint);
    if(it == xpoints_.end()){
        return {};
    }
    // dfs returns a pointer to the Xpoint where the cycle begins
    auto cycleXpoint = dfs(it->second);

    // no cycle was found
    if(cycleXpoint == nullptr){
        return {};
    }

    std::vector<Coord> path;
    std::shared_ptr<Xpoint> u = cycleXpoint;

    while(u != nullptr){
        Coord coord = u->coord;
        path.push_back(coord);
        u = u->pi;
    }

    // Checks if the path has to be reversed
    path.insert(path.begin(), path.back());
    if(path.begin() > path.end()){
        std::reverse(path.begin(), path.end());
    }
    return path;
}

Cost Datastructures::trim_fibre_network()
{
    // Replace this with your implementation
    return NO_COST;
}

std::pair<Coord, Coord> Datastructures::make_pair(Coord xpoint1, Coord xpoint2)
{
    std::pair<Coord, Coord> coordPair;
        if (xpoint1 < xpoint2)
        {
            coordPair.first = xpoint1;
            coordPair.second = xpoint2;
        }
        else
        {
            coordPair.first = xpoint2;
            coordPair.second = xpoint1;
        }
        return coordPair;
}

bool Datastructures::dijkstra(std::shared_ptr<Xpoint> start,
                              std::shared_ptr<Xpoint> end)
{
    // Resets the Xpoints to the default settings
    reset_xpoints();
    std::priority_queue<std::shared_ptr<Xpoint>,
    std::vector<std::shared_ptr<Xpoint>>, XpointCmp> queue;
    start->color = "gray";
    start->d = 0;
    queue.push(start);
    while(not queue.empty()){
        auto u = queue.top();
        queue.pop();
        // The path is ready
        if(u == end){
            return true;
        }
        for(auto v : u->fibres){
            std::pair<Coord, Coord> cPair = make_pair(u->coord,v.first->coord);
            Cost w = fibres_.find(cPair)->second->cost;
            relax(u, v.first, w);
            if(v.first->color == "white"){
                v.first->color = "gray";
                queue.push(v.first);
            }
        }
        u->color = "black";
    }
    return false;
}

void Datastructures::relax(std::shared_ptr<Xpoint> u,
                           std::shared_ptr<Xpoint> v, Cost w)
{
    if(v->d > u->d + w or v->d == -1){
        v->d = u->d + w;
        v->pi = u;
    }
}

void Datastructures::reset_xpoints()
{
    for(auto x : xpoints_){
        x.second->d = -1;
        x.second->pi = nullptr;
        x.second->color = "white";
    }
}

std::shared_ptr<Xpoint> Datastructures::dfs(std::shared_ptr<Xpoint> start)
{
    // Resets the xpoints to default settings
    reset_xpoints();
    std::stack<std::shared_ptr<Xpoint>> stack;
    stack.push(start);
    while(not stack.empty()){
        auto u = stack.top();
        stack.pop();
        if(u->color == "white"){
            u ->color = "gray";
            stack.push(u);
            for(auto v : u->fibres){
                if(v.first->color == "white"){
                    v.first->pi = u;
                    stack.push(v.first);
                }
                // The cycle is found and cycle point returned
                else if(v.first->color == "gray" and u->pi != v.first){
                    v.first->pi = nullptr;
                    return u;
                }
            }
        }
        else {
            u->color = "black";
        }
    }
    return nullptr;
}

bool Datastructures::bfs(std::shared_ptr<Xpoint> start, std::shared_ptr<Xpoint> end)
{
    // Resets the xpoints to default settings
    reset_xpoints();
    std::queue<std::shared_ptr<Xpoint>> queue;
    start->color = "gray";
    start->d = 0;
    queue.push(start);
    while(not queue.empty()){
        std::shared_ptr<Xpoint> u = queue.front();
        queue.pop();
        // the path is ready
        if(u == end){
            return true;
        }
        for(auto v : u->fibres){
            if(v.first->color == "white"){
                v.first->color = "gray";
                std::pair<Coord, Coord> cPair = make_pair(u->coord,v.first->coord);
                Cost w = fibres_.find(cPair)->second->cost;
                v.first->d = u->d + w;
                v.first->pi = u;
                queue.push(v.first);
            }
        }
        u->color = "black";
    }
    return false;
}

bool Datastructures::XpointCmp::operator()(std::shared_ptr<Xpoint> x1,
                                           std::shared_ptr<Xpoint> x2)
{
     return x1->d > x2->d;
}

#include <AStar.hpp>
#include <algorithm>
#include <iostream>

using namespace std::placeholders;

bool AStar::Vec2i::operator==(const Vec2i& coordinates_) {
  return (x == coordinates_.x && y == coordinates_.y);
}

AStar::Vec2i operator+(const AStar::Vec2i& left_, const AStar::Vec2i& right_) {
  return {left_.x + right_.x, left_.y + right_.y};
}

AStar::Node::Node(Vec2i coordinates_, Node* parent_) {
  parent = parent_;
  coordinates = coordinates_;
  G = H = 0;
}

AStar::uint AStar::Node::getScore() { return G + H; }

AStar::Generator::Generator() {
  setDiagonalMovement(false);
  setHeuristic(&Heuristic::manhattan);
  direction = {{0, 1},   {1, 0}, {0, -1}, {-1, 0},
               {-1, -1}, {1, 1}, {-1, 1}, {1, -1}};
}

void AStar::Generator::setWorldSize(Vec2i worldSize_) {
  worldSize = worldSize_;
  clearCache();
}

void AStar::Generator::setDiagonalMovement(bool enable_) {
  directions = (enable_ ? 8 : 4);
  clearCache();
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_) {
  heuristic = std::bind(heuristic_, _1, _2);
  clearCache();
}

void AStar::Generator::addCollision(Vec2i coordinates_) {
  walls.push_back(coordinates_);
  clearCache();
}

void AStar::Generator::removeCollision(Vec2i coordinates_) {
  auto it = std::find(walls.begin(), walls.end(), coordinates_);
  if (it != walls.end()) {
    walls.erase(it);
  }
  clearCache();
}

void AStar::Generator::clearCollisions() {
  walls.clear();
  clearCache();
}

void AStar::Generator::printMap(Vec2i source_, Vec2i target_) {
  for (int x = 0; x < worldSize.x; x++) {
    for (int y = 0; y < worldSize.y; y++) {
      if (detectCollision({x, y})) {
        if (x == source_.x && y == source_.y)
          std::cout << "s!";
        else if (x == target_.x && y == target_.y)
          std::cout << "g!";
        else
          std::cout << "o";
      } else if (x == source_.x && y == source_.y) {
        std::cout << "S";
      } else if (x == target_.x && y == target_.y) {
        std::cout << "G";
      } else {
        std::cout << "+";
      }
    }
    std::cout << std::endl;
  }
}

AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_) {
  return findPath_(source_, target_);
}

AStar::CoordinateList AStar::Generator::findPathCached(Vec2i source_,
                                                       Vec2i target_,
                                                       bool omit_start_end) {
  // check cache first
  auto cached = getCache(source_, target_);
  if (cached.size()) {
    // std::cout << "cache hit! \t";
    return cached;
  }

  cache_lock_ = true;
  bool start_collision, end_collision;
  AStar::CoordinateList result;
  start_collision = end_collision = false;
  if (omit_start_end) {
    start_collision = detectCollision(source_);
    end_collision = detectCollision(target_);
    removeCollision(source_);
    removeCollision(target_);
  }
  result = findPath_(source_, target_);
  if (omit_start_end) {
    if (start_collision) addCollision(source_);
    if (end_collision) addCollision(target_);
  }

  setCache(source_, target_, result);
  cache_lock_ = false;
  return result;
}

AStar::CoordinateList AStar::Generator::findPath_(Vec2i source_,
                                                  Vec2i target_) {
  Node* current = nullptr;
  NodeSet openSet, closedSet;
  openSet.insert(new Node(source_));

  while (!openSet.empty()) {
    current = *openSet.begin();
    for (auto node : openSet) {
      if (node->getScore() <= current->getScore()) {
        current = node;
      }
    }

    if (current->coordinates == target_) {
      break;
    }

    closedSet.insert(current);
    openSet.erase(std::find(openSet.begin(), openSet.end(), current));

    for (uint i = 0; i < directions; ++i) {
      Vec2i newCoordinates(current->coordinates + direction[i]);
      if (detectCollision(newCoordinates) ||
          findNodeOnList(closedSet, newCoordinates)) {
        continue;
      }

      uint totalCost = current->G + ((i < 4) ? 10 : 14);

      Node* successor = findNodeOnList(openSet, newCoordinates);
      if (successor == nullptr) {
        successor = new Node(newCoordinates, current);
        successor->G = totalCost;
        successor->H = heuristic(successor->coordinates, target_);
        openSet.insert(successor);
      } else if (totalCost < successor->G) {
        successor->parent = current;
        successor->G = totalCost;
      }
    }
  }

  CoordinateList path;
  while (current != nullptr) {
    path.push_back(current->coordinates);
    current = current->parent;
  }

  releaseNodes(openSet);
  releaseNodes(closedSet);

  return path;
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_,
                                              Vec2i coordinates_) {
  for (auto node : nodes_) {
    if (node->coordinates == coordinates_) {
      return node;
    }
  }
  return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_) {
  for (auto it = nodes_.begin(); it != nodes_.end();) {
    delete *it;
    it = nodes_.erase(it);
  }
}

bool AStar::Generator::detectCollision(Vec2i coordinates_) {
  if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
      coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
      std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
    return true;
  }
  return false;
}

void AStar::Generator::setCache(Vec2i source_, Vec2i target_,
                                CoordinateList path_) {
  path_cache.insert({{source_.x, source_.y}, {target_.x, target_.y}}, path_);
}

AStar::CoordinateList AStar::Generator::getCache(Vec2i source_, Vec2i target_) {
  auto path = path_cache.get({{source_.x, source_.y}, {target_.x, target_.y}})
                  .get_value_or(CoordinateList());
  return path;
}

void AStar::Generator::clearCache() {
  if (!cache_lock_) path_cache.clear();
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_) {
  return {abs(source_.x - target_.x), abs(source_.y - target_.y)};
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_) {
  auto delta = std::move(getDelta(source_, target_));
  return static_cast<uint>(10 * (delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_) {
  auto delta = std::move(getDelta(source_, target_));
  return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_) {
  auto delta = std::move(getDelta(source_, target_));
  return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}

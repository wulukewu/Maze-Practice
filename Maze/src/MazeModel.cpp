#include "MazeController.h"
#include "MazeModel.h"
#include "MazeView.h"

#include <chrono>
#include <random>
#include <algorithm>
#include <stack>
#include <queue>
#include <array>
#include <utility>

MazeModel::MazeModel(uint32_t height, uint32_t width)
    : maze{ height, std::vector<MazeElement>{ width, MazeElement::GROUND } }, solve_cost__(0), solve_cell__(0)
{
  maze[BEGIN_Y][BEGIN_X] = MazeElement::BEGIN;
  maze[END_Y][END_X] = MazeElement::END;
}

void MazeModel::setController(MazeController *controller_ptr)
{
  this->controller_ptr__ = controller_ptr;
}

int32_t MazeModel::getSolveCost() const
{
  return solve_cost__;
}

int32_t MazeModel::getSolveCell() const
{
  return solve_cell__;
}

void MazeModel::emptyMap()
{
  for (auto &row : maze)
    std::fill(row.begin(), row.end(), MazeElement::GROUND);

  controller_ptr__->enFramequeue(maze);
  setFlag__();
}

/* --------------------maze generation methods -------------------- */

/**
 * @brief Generate the maze by Randomized Prim's algorithm.
 * @param actions Two types of actions are supported:
 *        - G_PRIM: generate the maze by Randomized Prim's algorithm without breaking any wall.
 *        - G_PRIM_BREAK: after generating the maze by Randomized Prim's algorithm, it would break some walls randomly.
 */
void MazeModel::generateMazePrim(const MazeAction actions)
{
  initMaze__();

  std::mt19937 gen(std::chrono::high_resolution_clock::now().time_since_epoch().count());
  std::array<int32_t, 4> direction_order{ 0, 1, 2, 3 };
  std::vector<MazeNode> break_list;    // list for breaking wall
  std::vector<MazeNode> candidate_list;    // list for wall candidate

  {
    MazeNode seed_node;
    setBeginPoint__(seed_node);    // the begin point for the maze generation

    std::shuffle(direction_order.begin(), direction_order.end(), gen);
    for (const int32_t index : direction_order) {
      const auto [dir_y, dir_x] = dir_vec[index];
      if (inWall__(seed_node.y + dir_y, seed_node.x + dir_x))
        candidate_list.emplace_back(MazeNode{ seed_node.y + dir_y, seed_node.x + dir_x, maze[seed_node.y + dir_y][seed_node.x + dir_x] });    // add the wall around the begin point to the candidate list
    }
  }

  while (!candidate_list.empty()) {
    std::uniform_int_distribution<> wall_dis(0, candidate_list.size() - 1);
    int32_t random_index = wall_dis(gen);
    MazeNode current_node = candidate_list[random_index];    // pick one wall out
    MazeElement up_element{ MazeElement::INVALID }, down_element{ MazeElement::INVALID }, left_element{ MazeElement::INVALID }, right_element{ MazeElement::INVALID };    // the element of the wall's up, down, left, right
    // if the wall is confirmed to be a wall, then check its up, down, left, right
    if (current_node.element == MazeElement::WALL) {
      if (inWall__(current_node.y - 1, current_node.x))
        up_element = maze[current_node.y - 1][current_node.x];
      if (inWall__(current_node.y + 1, current_node.x))
        down_element = maze[current_node.y + 1][current_node.x];
      if (inWall__(current_node.y, current_node.x - 1))
        left_element = maze[current_node.y][current_node.x - 1];
      if (inWall__(current_node.y, current_node.x + 1))
        right_element = maze[current_node.y][current_node.x + 1];

      // if the wall's up and down are explored, or the wall's left and right are explored, then add this wall to the break list
      if ((up_element == MazeElement::EXPLORED && down_element == MazeElement::EXPLORED) || (left_element == MazeElement::EXPLORED && right_element == MazeElement::EXPLORED)) {
        break_list.emplace_back(current_node);
        candidate_list.erase(candidate_list.begin() + random_index);
      }
      else {
        // else, set the wall to be explored
        current_node.element = MazeElement::EXPLORED;
        maze[current_node.y][current_node.x] = MazeElement::EXPLORED;
        candidate_list.erase(candidate_list.begin() + random_index);

        controller_ptr__->enFramequeue(maze, current_node);    // append the maze to the framequeue

        // changed the current node to the next node
        if (up_element == MazeElement::EXPLORED && down_element == MazeElement::GROUND)
          ++current_node.y;
        else if (up_element == MazeElement::GROUND && down_element == MazeElement::EXPLORED)
          --current_node.y;
        else if (left_element == MazeElement::EXPLORED && right_element == MazeElement::GROUND)
          ++current_node.x;
        else if (left_element == MazeElement::GROUND && right_element == MazeElement::EXPLORED)
          --current_node.x;

        current_node.element = MazeElement::EXPLORED;
        maze[current_node.y][current_node.x] = MazeElement::EXPLORED;
        std::shuffle(direction_order.begin(), direction_order.end(), gen);

        // iterate the new node's up, down, left, right
        for (const int32_t index : direction_order) {
          const auto [dir_y, dir_x] = dir_vec[index];

          // if the node is in the maze and the node is a wall
          if (inWall__(current_node.y + dir_y, current_node.x + dir_x)) {
            if (maze[current_node.y + dir_y][current_node.x + dir_x] == MazeElement::WALL)
              candidate_list.emplace_back(MazeNode{ current_node.y + dir_y, current_node.x + dir_x, maze[current_node.y + dir_y][current_node.x + dir_x] });
          }
        }

        controller_ptr__->enFramequeue(maze, current_node);    // append the maze to the framequeue
      }
    }
  }

  // if the action is G_PRIM_BREAK, then break some walls
  if (actions == MazeAction::G_PRIM_BREAK) {
    std::shuffle(break_list.begin(), break_list.end(), gen);
    std::uniform_int_distribution<> dis(0, break_list.size() / 5 - 1);    // at most break 1/5 walls
    for (int i = dis(gen); i > 0; --i) {
      std::uniform_int_distribution<> wall_dis(0, break_list.size() - 1);
      int32_t random_index = wall_dis(gen);
      maze[break_list[random_index].y][break_list[random_index].x] = MazeElement::GROUND;
      break_list.erase(break_list.begin() + random_index);
    }
  }

  setFlag__();
  cleanExplored__();
  controller_ptr__->setModelComplete();
}    // end generateMazePrim()

/**
 * @brief Generate the maze by Recursive Backtracker algorithm, also known as Randomized depth-first search algorithm.
 */
void MazeModel::generateMazeRecursionBacktracker()
{
  initMaze__();

  // TODO: implement your algorithm here

  setFlag__();
  cleanExplored__();
  controller_ptr__->setModelComplete();
}    // end generateMazeRecursionBacktracker()


/**
 * @brief Generate the maze by Recursive Division algorithm.
 * @param uy The upper y-coordinate of target area.
 * @param lx The left x-coordinate of target area.
 * @param dy The lower y-coordinate of target area.
 * @param rx The right x-coordinate of target area.
 * @param is_first_call Whether it is the first call of the function, used to initialize the wall and set the ModelComplete flag.
 */
void MazeModel::generateMazeRecursionDivision(const int32_t uy, const int32_t lx, const int32_t dy, const int32_t rx, bool is_first_call)
{
  // Hint: in mine implementation, it was implemented via resursive,
  // thus it needs an flag to determine if it's the first call of the function to init and reset maze.
  // You can check the implementation of `solveMazeDFS` below.
  // Feel free to change anything here, just remeber freeup the flag and set the model complete flag.
  if (is_first_call)
    resetWallAroundMaze__();

  // TODO: implement your algorithm here

  if (is_first_call) {
    setFlag__();
    cleanExplored__();
    controller_ptr__->setModelComplete();
  }
}    // end generateMazeRecursionDivision()

/* --------------------maze solving methods -------------------- */

/**
 * @brief Solve the maze by DFS algorithm.
 * @param y The y-coordinate of the current node.
 * @param x The x-coordinate of the current node.
 * @param is_first_call Whether it is the first call of the function, used to initialize the explored and set the ModelComplete flag.
 * @return Whether the maze is solved.
 */
bool MazeModel::solveMazeDFS(const int32_t y, const int32_t x, bool is_first_call)
{
  if (maze[y][x] == MazeElement::END)
    return true;

  if (is_first_call) {
    cleanExplored__();
    solve_cost__ = 0;
    solve_cell__ = 0;
  }
  else {
    maze[y][x] = MazeElement::EXPLORED;
    controller_ptr__->enFramequeue(maze, MazeNode{ y, x, MazeElement::EXPLORED });
  }

  // iterate the current node's up, down, left, right
  for (const auto &[dir_y, dir_x] : dir_vec) {
    const int32_t target_y = y + dir_y;
    const int32_t target_x = x + dir_x;

    if (!inMaze__(target_y, target_x))
      continue;

    if (maze[target_y][target_x] != MazeElement::GROUND && maze[target_y][target_x] != MazeElement::END)
      continue;

    // keep searching the next node via recursion call DFS
    if (solveMazeDFS(target_y, target_x)) {
      if (maze[target_y][target_x] != MazeElement::END) {
        maze[target_y][target_x] = MazeElement::ANSWER;
        controller_ptr__->enFramequeue(maze);    // append the maze to the framequeue, I dont want to render the update node here so only one argument here

        solve_cost__++;
        solve_cell__++;
      }

      if (is_first_call)
        controller_ptr__->setModelComplete();

      return true;
    }
  }

  // If we get here, no path was found, it should not happen
  if (is_first_call)
    controller_ptr__->setModelComplete();

  return false;
}    // end solveMazeDFS()

/**
 * @brief Solve the maze by BFS algorithm.
 */
void MazeModel::solveMazeBFS()
{
  cleanExplored__();
  solve_cost__ = 0;
  solve_cell__ = 0;

  // TODO: implement your algorithm here

  // If we get here, no path was found, it should not happen
  controller_ptr__->setModelComplete();
}    // end solveMazeBFS()

/**
 * @brief Solve the maze by A* algorithm, because the UCS and Greedy best-first search algorithm are special cases of A* algorithm, so I merge them into one function to solve the maze.
 * @param actions The action of the maze solving algorithm.
 *        - S_ASTAR_MANHATTAN: A* algorithm with Manhattan distance as heuristic function
 *        - S_ASTAR_EUCLIDEAN: A* algorithm with Euclidean distance as heuristic function
 *        - S_GREEDY_MANHATTAN: Greedy best-first search algorithm with Manhattan distance as heuristic function
 *        - S_GREEDY_TWO_NORM: Greedy best-first search algorithm with 2-norm distance as heuristic function
 *        - S_UCS: Uniform-cost search algorithm
 */
void MazeModel::solveMazeAStar(const MazeAction actions)
{
  cleanExplored__();
  solve_cost__ = 0;
  solve_cell__ = 0;

  // TODO: implement your algorithm here

  // If we get here, no path was found, it should not happen
  controller_ptr__->setModelComplete();
}    // end solveMazeAStar()

/* -------------------- private utility function --------------------   */

/**
 * @brief Clean the explored and answer status in the maze.
 */
void MazeModel::cleanExplored__()
{
  for (auto &row : maze) {
    std::replace(row.begin(), row.end(), MazeElement::EXPLORED, MazeElement::GROUND);
    std::replace(row.begin(), row.end(), MazeElement::ANSWER, MazeElement::GROUND);
  }

  controller_ptr__->enFramequeue(maze);
  setFlag__();
}

/**
 * @brief Initialize the maze, set the wall and ground into grid-shaped.
 */
void MazeModel::initMaze__()
{
  for (int32_t y{}; y < MAZE_HEIGHT; ++y) {
    for (int32_t x{}; x < MAZE_WIDTH; ++x) {
      if (y == 0 || y == MAZE_HEIGHT - 1 || x == 0 || x == MAZE_WIDTH - 1)
        maze[y][x] = MazeElement::WALL;
      else if (x % 2 == 1 && y % 2 == 1)    // odd for ground
        maze[y][x] = MazeElement::GROUND;
      else
        maze[y][x] = MazeElement::WALL;
    }
  }

  controller_ptr__->enFramequeue(maze);
}

/**
 * @brief Reset the wall around the maze.
 */
void MazeModel::resetWallAroundMaze__()
{
  for (int32_t y = 0; y < MAZE_HEIGHT; ++y) {
    for (int32_t x = 0; x < MAZE_WIDTH; ++x) {
      if (x == 0 || x == MAZE_WIDTH - 1 || y == 0 || y == MAZE_HEIGHT - 1)
        maze[y][x] = MazeElement::WALL;    // Wall
      else
        maze[y][x] = MazeElement::GROUND;    // Ground
    }
  }

  controller_ptr__->enFramequeue(maze);
}

/**
 * @brief Set the begin and end flag in the maze.
 */
void MazeModel::setFlag__()
{
  maze[BEGIN_Y][BEGIN_X] = MazeElement::BEGIN;
  maze[END_Y][END_X] = MazeElement::END;

  controller_ptr__->enFramequeue(maze);
}

/**
 * @brief Check if the node is in the wall, which means (y < MAZE_HEIGHT - 1) && (x < MAZE_WIDTH - 1) && (y > 0) && (x > 0)
 * @param y The y-coordinate of the node.
 * @param x The x-coordinate of the node.
 * @return Whether the node is in the wall.
 */
bool MazeModel::inWall__(const int32_t y, const int32_t x)
{
  return (y < MAZE_HEIGHT - 1) && (x < MAZE_WIDTH - 1) && (y > 0) && (x > 0);
}

/**
 * @brief Check if the node is in the maze, which means (y < MAZE_HEIGHT) && (x < MAZE_WIDTH) && (y > -1) && (x > -1)
 * @param y The y-coordinate of the node.
 * @param x The x-coordinate of the node.
 * @return Whether the node is in the maze.
 */
bool MazeModel::inMaze__(const int32_t y, const int32_t x)
{
  return (y < MAZE_HEIGHT) && (x < MAZE_WIDTH) && (y > -1) && (x > -1);
}

/**
 * @brief generate a random begin point for maze generation algorithm
 *
 * @param seed_y
 * @param seed_x
 */
void MazeModel::setBeginPoint__(MazeNode &node)
{
  std::mt19937 gen(std::chrono::high_resolution_clock::now().time_since_epoch().count());
  std::uniform_int_distribution<> y_dis(0, (MAZE_HEIGHT - 3) / 2);
  std::uniform_int_distribution<> x_dis(0, (MAZE_WIDTH - 3) / 2);

  node.y = 2 * y_dis(gen) + 1;
  node.x = 2 * x_dis(gen) + 1;
  node.element = MazeElement::EXPLORED;
  maze[node.y][node.x] = MazeElement::EXPLORED;    // set the randomly chosen point as the generation start point

  controller_ptr__->enFramequeue(maze);
}
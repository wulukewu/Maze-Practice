# Maze-Practice

This is a project for MCL students to practice C++. Some functions have been left unimplemented, and the complete implementation can be found in *[Mase](https://github.com/Mes0903/Mase)*.

*Mase* is an cross platform project for the visualization of maze generalization and maze solver algorithm.

It was made by C++17 and ImGui, with the OpenGL and glfw/glad backend.

![](document/mase.gif)

> There was an Qt version before but no longer maintain anymore.

# For students

This project follows the MVC (Model-View-Controller) design pattern. Inside the "Maze" folder, you'll find several files corresponding to Model, View, and Controller.

- The Model is responsible for the main algorithm calculations.
- The View handles rendering the visuals.
- The Controller manages the interaction between the Model and View to ensure the program runs properly.

Our exercise aims to let you implement the algorithms in the Model, so you do not need to worry too much about the details of View and Controller. You can still successfully complete the project while focusing primarily on the Model.

Files `MazeModel.h` and `MazeModel.cpp` contain all the details of the Model algorithms and utility functions, while `MazeDefine.h` includes some parameters that you will use.

To draw the maze, you will need to interact with the Controller. Don't worry — we have minimized the required work for this. In the MazeModel class, there is a pointer called `controller_ptr__` for communication with the Controller. You only need to call the function `controller_ptr__->enFramequeue` within your algorithm in the Model to update the display.

I've provided two implementations as examples: `generateMazePrim` and `solveMazeDFS`. One is a maze generation algorithm, and the other is a maze-solving algorithm. You can see how to use `controller_ptr__->enFramequeue` in these examples.

Additionally, each algorithm in the Model must set a complete flag before finishing. You will find that we call `setModelComplete` for this purpose in the current code — please do not remove this call.

Since this course is self-study and doesn't provide any credits, you are free to look at my full implementations if you get stuck. However, I do not recommend doing so unless you have been struggling for a long time, as turning your own ideas into code is an essential skill to develop.

As for the algorithms themselves, I wrote a simple introduction in Chinese four years ago, which you can find in `Maze/README.md`. Alternatively, you can refer to the links at the end of this `README`. These are all classic algorithms, so you can easily find other implementations on GitHub. We also welcome any new explanations you may want to contribute.

Finally, since we will not be grading your assignments, as long as you can get the project running, you are free to modify any files in this project as you see fit.

If you have any questions, please feel free to ask me, and I will assist you to the best of my ability.

# Implemented algorithm

For generalizing maze, there are three implementations now:

- [Randomized Prim's algorithm](https://en.wikipedia.org/wiki/Maze_generation_algorithm#Iterative_randomized_Prim's_algorithm_(without_stack,_without_sets))
- [Randomized depth-first search](https://en.wikipedia.org/wiki/Maze_generation_algorithm#Randomized_depth-first_search)
- [Recursive division method](https://en.wikipedia.org/wiki/Maze_generation_algorithm#Recursive_division_method)

For solving maze, there are five implementations now:

- [BFS](https://en.wikipedia.org/wiki/Breadth-first_search)
- [DFS](https://en.wikipedia.org/wiki/Depth-first_search)
- [UCS(uniform cost search)](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm#Practical_optimizations_and_infinite_graphs)
- [Greedy](https://en.wikipedia.org/wiki/Greedy_algorithm)
- [A\*](https://en.wikipedia.org/wiki/A*_search_algorithm)

There are two heuristic functions now:

- [Euclidean distance(two norm)](https://en.wikipedia.org/wiki/Euclidean_distance)
- [Taxicab geometry](https://en.wikipedia.org/wiki/Taxicab_geometry)

# Dependencies

- OpenGL
- C++17

# How to build

```bash
git clone https://github.com/OpenMCL/Maze-Practice.git
cd Maze-Practice
git submodule init
git submodule update
mkdir build && cd build
cmake ..
cmake --build .
```

For Doxygen:

```bash
cd document
doxygen ./Doxyfile
```

## wsl

if you are using WSL as your environment, you may encounter the wayland-scanner error:

> ... failed to find wayland-scanner

In this case, you can uncomment the cmake flag for glfw to disable wayland:

```cmake
# in the 3rdparty/CMakeLists.txt

if(UNIX)
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
endif()

add_subdirectory(glfw)
# ... the remain content
```

## glad

Glad in this project is for opengl 4.6, so you may need to change the version of glad if you are using a different version of OpenGL.

You can download the correspond version on the [glad loader website](https://glad.dav1d.de/).

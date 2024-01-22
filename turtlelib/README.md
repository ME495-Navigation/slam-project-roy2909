# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input
- svg - Handles visulaization of 2D geometry

# Conceptual Questions
1. If you needed to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
   - The three ways of implemeting the normalize functionality are:
     1. As a member function within a class
     2. Free function with an output parameter
     3. As a free independent function as an input argument

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
   - The first method is made only if it needs direct access to the representation of a class, according to C.4 `Make a function a member only if it needs direct access to the representation of a class`. It is simple to use and members can directly acces the private members of the class but can lead to unexpected behaviour if the user doesn't know that the object s being modified.
   - This will not change the internal state directly but will require user to handle an additional output parameter.
   - This is easy to implement and works well for an interface to the Vector2D struct, but it cannot access private memebers of the class 

   - Which of the methods would you implement and why?
      I would implement it as number 3. i.e a free function as it is simplier and does not need any member variables of a class.

2. What is the difference between a class and a struct in C++?'
   The main differnce between class and struct is the member access control and default visibility. Class memebers are private by default while structs are public by default.


3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?
   According to C.2`Use class if the class has an invariant; use struct if the data members can vary independently`, The elements of the Vector2D are independent whereas in the Transform2D elemets are dependent on each other. The entire tranfomation matrix can change based on a single element.
   According to C.4 `Make a function a member only if it needs direct access to the representation of a class` Transform2D functions like translation() and rotation() require access to the internal state of class. Here the struct is used to group and store data together.


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
According to `C.46: By default, declare single-argument constructors explicit` . This is done to prevent implicit type conversion.

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer
`According to Con.1: By default, make objects immutable`

Immutable objects are easier to reason about, so make objects non-const only when there is a need to change their value. 
Here Transform2D::inv() returns a new Transform2D object whearas Transform2D::operator*=() modifies the object itself. 
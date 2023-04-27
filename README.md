Collection of helper functions for simulations, mathematical operations using Eigen, building of block-structured matrices and much more...

## Logger
This is a `Logger` class used to collect a list of variables to log into csv files.
### Example
```cpp
#include <cpp-utils/simulation.h>

Eigen::MatrixXd m(2,3);
m << 1, 2, 3,
     4, 5, 6;
Eigen::Vector2d v = {2., 1.5};
double s = 3.14;
     
utils::Logger logger(realpath("data/", nullptr)); // Insert local path where to create files (must have been created prior to this)
logger.appendSuffix("toappend"); //
logger.add(m, "mymatrix");
logger.add(v, "myvector", true);
logger.add(s, "myscalar");

logger.logAll();

v(0) = 5;

logger.logAll();
```

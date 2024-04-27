# QuITO v.2 : Quasi-Interpolation based Trajectory Optimization 
 
QuITO v.2 is a MATLAB-based numerical software package for solving constrained nonlinear optimal control problems via direct optimization. The base algorithm is a novel direct multiple shooting technique that employs a special kind of quasi-interpolation technique on piecewise uniform grid to approximate the control trajectory. The change-point localization and the mesh refinement modules in QuITO v.2 automatically detects irregular regions in the numerical optimal control and iteratiely refine the time-grid until a presepecified tolerance in the minimzed cost is reached. Currently, QuITO can handle the following problem specifications: 

* Dynamics (ODEs): linear, nonlinear;
* Constraint: state, control, mixed (which can be a nonlinear function of states and control);
* Boundary specification: boundary values, constraints of terminal states.

The following table collects all the pre-loaded optimal control problem examples in QuITO:

| Problem | Dynamics | State/Control/Mixed Constraints | Singular Control | Additional Function |
| ------------- | ------------- | -------------------- | ---------------- | ------------------- |
| [Aly Chan](./examples/Aly%20Chan/)  | Nonlinear  |       No, Yes, No   | No | |
| [Bang Bang problem](./examples/Bang%20Bang%20problem)  | Nonlinear  |       No, Yes, No   | No | Mesh Refinement |
| [Bressan problem](./examples/Bressan%20problem)  | Nonlinear  |       No, Yes, No   | Yes | Mesh Refinement |
| [Bryson Denham](./examples/Bryson%20Denham/)  | Linear  |    Yes, No, No   | No | |
| [Catalyst Mixing problem](./examples/Catalyst%20mixing%20problem)  | Nonlinear  |       No, Yes, No   | Yes | Mesh Refinement |
| [Double integrator tracking](./examples/Double%20integrator%20tracking/)  | Linear  |    Yes, Yes, No  | No | |       
| [Inverted pendulum on a cart](./examples/Inverted%20pendulum%20on%20a%20cart/)  | Linear  | Yes, Yes, No  | No | |
| [Multi-agent AUV path planning](./examples/AUV%20path%20planning)  | Nonlinear  |       Yes, Yes, No   | No | Mesh Refinement |
| [Rayleigh problem (with control constraints)](./examples/Rayleigh%20problem%20(with%20control%20constraints)/)  | Nonlinear  | No, Yes, No  | No | |
| [Rayleigh problem (with mixed constraints)](./examples/Rayleigh%20problem%20(with%20mixed%20constraints)/)   | Nonlinear  | No, No, Yes | No | |
| [Robot Path Planning](./examples/Robot%20Path%20Planning/)  | Linear  | Yes, No, No  | No | |
| [SIRI problem](./examples/SIRI%20problem)  | Nonlinear  |       Yes, Yes, No   | Yes | Mesh Refinement |
| [VanDerPol control constrainted](./examples/VanDerPol%20control%20constrained/)  | Nonlinear  | No, Yes, No  | Yes | |
| [VanDerPol state constrainted](./examples/VanDerPol%20state%20constrained/)  | Nonlinear  | Yes, No, No  | No | |

Implementation details for all the individual examples are given in their README.md files which can be found within their directories. Simply click and scroll down.

## How to cite?
In case you're using QuITO v.2 (or it's previous version https://github.com/chatterjee-d/QuITO.git), consider citing the articles: 
1) S.Ganguly, N. Randad, D. Chatterjee, R. Banavar, Constrained trajectory synthesis via quasi-interpolation, IEEE CDC 2022, https://ieeexplore.ieee.org/abstract/document/9992892
2) S.Ganguly, N. Randad, R. A. D'Silva, M. Raj, D. Chatterjee, QuITO: Numerical software for constrained nonlinear optimal control problems, https://www.sciencedirect.com/science/article/pii/S2352711023002534.
3) S.Ganguly, R. A. D'Silva D. Chatterjee, QuITO v.2: Trajectory Optimization with Uniform Error Guarantees under Path Constraints https://doi.org/10.48550/arXiv.2404.13681

## Contributors

1) [Rihan Aaron D'Silva](https://sites.google.com/view/rihanaarondsilva)
2) [Siddhartha Ganguly](https://sites.google.com/view/siddhartha-ganguly)
3) [Debasish Chatterjee](https://www.sc.iitb.ac.in/~chatterjee/master/homepage/index.html)

---

## How to install?

### Step I: install CasADi
QuITO uses CasADi, an open-source framework for nonlinear optimization, for its optimization routine. CasADi can be downloaded from here: https://web.casadi.org/get. 
- Once the .zip file is downloaded from the mentioned webpage, extract the contents onto a desired folder.  
- Navigate to that folder in MATLAB and execute the following commands in MATLAB Command Window:
```
addpath(genpath('<path to the folder in which CasADi files are extracted to>'));
import casadi.*;
``` 

To check CasADi installation, try executing the following lines in the MATLAB Command Window:
```
x = MX.sym('x')
disp(jacobian(sin(x),x))
```
If any error is thrown, this implies an irregular installation of CasADi, and the user is advised to follow the above instructions again closely. Also, please keep in mind that CasADi might have to be imported to MATLAB every time MATLAB is restarted, depending on the user's system settings.


Next, download and unzip the QuITO package source files into a  desired folder, and add the included [src](./src/) folder and its all subfolders to MATLAB's path directory. It can be done in the following ways:
- Access through the Set Path button on the Home ribbon and add the subfolders or,
- Open the package folder on MATLAB (which can be verified from the address field on the top). You can run `pwd` command on the MATLAB terminal to check whether the package directory is currently open. Then run the following command on the MATLAB terminal `addpath(genpath(fullfile(pwd, 'src')))` to add the [src](./src) folder and its subfolders to MATLAB’s path.

---

### Step II (using QuITO): running the pre-loaded examples and adding your own problem

You can run the pre-loaded examples, formulate your own problems, and then solve it by employing QuITO easily. There are two ways to achieve that: 

#### (a) Run the main.m file: 

The examples contain the following files: <br>

* example.m: contains system specifications, constraint profile, etc., specific to an example;
* options.m: contains integration schemes, solver options, etc.;
* postProcess.m: contains options for plotting the state-action trajectories;
* main.m: the main/primary file the user needs to run by specifying the number of steps $N>0$ and the shape parameter $D>0$.

To solve any example problem, the main.m file needs to be executed. We recommend the IPOPT (Interior Point Optimization) solver for nonlinear optimization problems and is already available within CasADI. For details, visit: https://coin-or.github.io/Ipopt.

Same steps need to be followed when the user wants to solve their own problems. Simply populate the example.m, options.m and postProcess,m files and execute the main.m file by setting the desirable parameters $(N,D)$. 

The other hassle free and quick option is to use the GUI. Have a look at the installation steps. 


#### (b) Use the Graphical User Interface (GUI)

#### Installation
The QuITO toolbox is packaged in the form of a MATLAB app with a Graphical Interface to ease the process of making use of the toolbox. To install the GUI, navigate to the [Graphical Interface](./Graphical%20Interface/) directory, and in that directory, find the installation file, [QuITO - Graphical Interface.mlappinstall](./Graphical%20Interface/QuITO%20-%20Graphical%20Interface.mlappinstall). Clicking on which MATLAB throws a pop-up prompting the user to install the application. Once the app is installed, the user can find it in the **APPS** section on MATLAB's toolbar on the top. 

#### Instructions to Use
1) Before starting the app, please ensure CasADI is installed and correctly imported onto MATLAB's path. The same can be verified using the commands `path` and `ver` on the Command Window.
2) Once verified, navigate to the base directory of QuITO, and open the MATLAB app named **QuITO** from the **APPS** section on MATLAB's toolbar at the top of the window.
3) Here, the user can select their preferred Example Problem or any User-Defined Template Problem from the dropdown, input values for _Number of Steps (N)_, _Shape Parameter (D)_, and _Generating function_, and simply click on the _Run_ button.
4) The user must ensure the current working directory on MATLAB is always the [base directory](./) of the QuITO toolbox.

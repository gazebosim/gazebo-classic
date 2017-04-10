# Introduction

Interested in developing new features, fixing bugs, or making any other
kind of contribution which touches Gazebo's codebase? Read on to find out how!

## Development process

We follow a development process designed to reduce errors, encourage
collaboration, and make high quality code. The process may seem rigid and
tedious, but every step is worth the effort.

### Steps to follow

1. Are you sure?

    Run through this mental checklist before getting started.

    1. Has your idea already been done, or maybe someone is already working on it?

        Check [answers.gazebosim.org](http://answers.gazebosim.org) and the [issue tracker](https://bitbucket.org/osrf/gazebo/issues).

    1. Get feedback from the Gazebo core team.

        Send an email to the
        [mailing list](https://groups.google.com/a/osrfoundation.org/forum/#!forum/gazebo),
        post a question on [Gazebo Answers](http://answers.gazebosim.org), or use the
        [issue tracker](https://bitbucket.org/osrf/gazebo/issues) to get feedback from
        Gazebo developers.

1. [Fork Gazebo](https://bitbucket.org/osrf/gazebo/fork)

    This will create your own personal copy of Gazebo. All of your development
    should take place in your fork.

1. Choose a base branch

    If your changes will break API or ABI, then base your new branch off of
    `default`. If your changes don't break API/ABI and you would like them
    to be released to an existing gazebo release with major version `N`,
    then use branch `gazeboN` as the base.

1. Work out of a branch

    Always work out of a new branch, never off of the base branch. This is a
    good habit to get in, and will make your life easier.

1. Write your code.

    This is the fun part.

1. Write tests

    In most cases, a pull request will only be accepted if it has tests. See the
    "Write tests" section below for more information.

1. Compiler warnings

    Code must have zero compile warnings, or at least make sure your pull
    request is not adding new warnings.

1. Style

    Static code checking analyzes your code for bugs, such as potential memory
    leaks, and style. Gazebo's static code checker uses `cppcheck`, and a
    modified `cpplint`. You'll need to install cppcheck on your system. Ubuntu
    users can install via:

        sudo apt-get install cppcheck

    To check your code, run the following script from the root of the Gazebo
    sources:

        sh tools/code_check.sh

    It takes a few minutes to run. Fix all errors and warnings until the output
    looks like:

        Total errors found: 0

    The tool does not catch all style errors. See the "Style" section below for
    more information.

1. Tests pass

    There must be no failing tests. You can check by running `make test` in
    your build directory.

    Running all tests in Gazebo may take a long time. If you expect your changes
    to only affect a few different tests, it should be enough to run each one
    of them individually from the build directory, for example:

        ./test/integration/INTEGRATION_world

1. Documentation.

    Document all your code. Every class, function, member variable must have
    doxygen comments. All code in source files must have documentation that
    describes the functionality. This will help reviewers and future developers.

1. Review your code.

    Before submitting your code through a pull request, take some time to
    review everything line-by-line. The review process will go much faster if
    you make sure everything is perfect before other people look at your code.
    There is a bit of the human-condition involved here. Folks are less likely
    to spend time reviewing your code if it's sloppy.

1. Small pull requests

    A large pull request is hard to review, and will take a long time. It is
    worth your time to split a large pull request into multiple smaller pull
    requests. For reference, here are a few examples:

    * [Small, very nice](https://bitbucket.org/osrf/gazebo/pull-request/1732)

    * [Medium, still okay](https://bitbucket.org/osrf/gazebo/pull-request/1700/)

    * [Too large](https://bitbucket.org/osrf/gazebo/pull-request/30)

1. [Make a pull request](https://bitbucket.org/osrf/gazebo/pull-request/new)

    Submit a pull request when you're ready.

1. Continuous integration

    The moment you make a pull request, a few jobs in our
    [continuous integration](http://build.osrfoundation.org/)
    server will be started. These jobs will build your branch on Linux, Mac and
    Windows, run all tests and check for warnings.

    Your pull request will be updated with the status of these builds. Take some
    time to check these builds and see if you've introduced test failures,
    warnings or broke some build. If you did and know how to fix it, do so. If
    you don't know, speak up and someone may try to help you.

1. Review

    At least two other people have to approve your pull request before it can
    be merged. Please be responsive to any questions and comments.

1. Done, phew.

    Once you have met all the requirements, you're code will be merged. Thanks
    for improving Gazebo!

### Internal Developers

This section is targeted mostly at people who have commit access to the main
repositories.

In addition to the general development process, please follow these steps
before submitting a pull request. Each step is pass/fail, where the test or
check must pass before continuing to the next step.

1. Run the style checker on your personal computer
1. Run all, or only relevant, tests on your personal computer
1. Run your branch through a Jenkins Linux no-gpu build
1. Run your branch through a Jenkins Linux Nvidia build
1. Run your branch through a Jenkins Homebrew build
1. Run your branch through a Jenkins Windows build
1. Run your branch through the ABI/API checker if targeting a release branch
1. Submit the pull request, and make sure the following are included
(a set of jenkins jobs will run automatically once the pull request is created,
if not included automatically, you must add them manually):
  1. Link to a coverage report
  1. Link to a passing Homebrew build
  1. Link to a passing Linux no-gpu build
  1. Link to a passing Linux Nvidia build
  1. Link to a passing Windows build
  1. Link to a passing ABI/API report if the pull request is targeted at a release branch

# Style

In general, we follow [Google's style guide](https://google.github.io/styleguide/cppguide.html). However, we add in some extras.

1. **This pointer**
> All class attributes and member functions must be accessed using the `this->` pointer. Here is an [example](https://bitbucket.org/osrf/gazebo/src/default/gazebo/physics/Base.cc#cl-40).

1. **Underscore function parameters**
> All function parameters must start with an underscore. Here is an [example](https://bitbucket.org/osrf/gazebo/src/default/gazebo/physics/Base.cc#cl-77).

1. **Do not cuddle braces**
> All braces must be on their own line. Here is an [example](https://bitbucket.org/osrf/gazebo/src/default/gazebo/physics/Base.cc#cl-131).

1. **Multi-line code blocks**
> If a block of code spans multiple lines and is part of a flow control statement, such as an `if`, then it must be wrapped in braces. Here is an [example](https://bitbucket.org/osrf/gazebo/src/default/gazebo/physics/Base.cc#cl-249)

1. **++ operator**
> This occurs mostly in `for` loops. Prefix the `++` operator, which is [slightly more efficient than postfix in some cases](http://programmers.stackexchange.com/questions/59880/avoid-postfix-increment-operator).

1. **PIMPL/Opaque pointer**
> If you are writing a new class, it must use a private data pointer. Here is an [example](https://bitbucket.org/osrf/gazebo/src/default/gazebo/physics/World.hh?at=default#cl-479), and you can read more [here](https://en.wikipedia.org/wiki/Opaque_pointer).

1. **const functions**
> Any class function that does not change a member variable should be marked as `const`. Here is an [example](https://bitbucket.org/osrf/gazebo/src/default/gazebo/physics/Entity.cc?at=default#cl-175).

1. **const parameters**
> All parameters that are not modified by a function should be marked as `const`. This applies to parameters that are passed by reference, pointer, and value. Here is an [example](https://bitbucket.org/osrf/gazebo/src/default/gazebo/physics/Entity.cc?at=default#cl-217).

1. **Pointer and reference variables**
> Place the `*` and `&` next to the varaible name, not next to the type. For example: `int &variable` is good, but `int& variable` is not. Here is an [example](https://bitbucket.org/osrf/gazebo/src/default/gazebo/physics/Entity.cc?at=default#cl-217).

1. **Camel case**
> In general, everything should use camel case. Exceptions include SDF element names, and protobuf variable names. Here is an [example](https://bitbucket.org/osrf/gazebo/src/default/gazebo/physics/Entity.cc?at=default#cl-217).

1. **Class function names**
> Class functions must start with a capital letter, and capitalize every word.
>
> `void MyFunction();` : Good
>
> `void myFunction();` : Bad
>
> `void my_function();` : Bad

1. **Variable names**
> Variables must start with a lower case letter, and capitalize every word thereafter.
>
> `int myVariable;` : Good
>
> `int myvariable;` : Bad
>
> `int my_variable;` : Bad

1. **No inline comments**
> `//` style comments may not be placed on the same line as code.
>
> `speed *= 0.44704;  // miles per hour to meters per second` : Bad

# Write tests

Gazebo uses [GTest](http://code.google.com/p/googletest) for general testing
and [QTest](http://doc.qt.io/qt-5/qtest.html) for GUI tests. There are a few
kinds of tests:

1. Unit tests: all classes should have corresponding unit tests. These live
in the same directory as the source code and are prefixed by `_TEST`.

1. Integration tests: tests which verify how many classes are working together
go under the `tests/integration` directory.

1. Regression tests: tests which fix broken features go under `tests/regression`
and are prefixed by the issue number on Gazebo's
[issue tracker](https://bitbucket.org/osrf/gazebo/issues).

Before creating a new integration or regression test file, check the current
test files. If one closely matches the topic of your new code, simply add a new
test function to the file. Otherwise, create a new test file, and write your
test.

## Test coverage

The goal is to achieve 100% line and branch coverage. However, this is not
always possible due to complexity issues, analysis tools misreporting
coverage, and time constraints. Try to write as complete of a test suite as
possible, and use the coverage analysis tools as guide. If you have trouble
writing a test please ask for help in your pull request.

Gazebo has a build target called `make coverage` that will produce a code
coverage report. You'll need to have
[lcov](http://ltp.sourceforge.net/coverage/lcov.php) and
[gcov](https://gcc.gnu.org/onlinedocs/gcc/Gcov.html) installed.

1. In your `build` folder, compile Gazebo with `-DCMAKE_BUILD_TYPE=Coverage`

        cmake -DCMAKE_BUILD_TYPE=Coverage ..\
        make

1. Run a single test, or all the tests

        make test

1. Make the coverage report

        make coverage

1. View the coverage report

        firefox coverage/index.html

## Gazebo assertions

### What is an assertion?

An assertion is a check, which always produces a boolean result, that
developers place in the code when want to be sure that check is always true.
They are aimed to detect programming errors and should check for impossible
situations in the code. If the assertion check failed, the assertion will
stop the program immediately.

     Object * p = some_crazy_function()
     GZ_ASSERT(p != nullptr, "Object from some_crazy_function should never point to NULL")
     p->run()

### Gazebo runtime assertions: GZ_ASSERT

In Gazebo, the GZ_ASSERT macro id designed to handle all our runtime assertions

     GZ_ASSERT(<condition to check>,<fail msg>)

* `condition-to-check`: anything returning a boolean value that should always be true.
* `fail msg`: message displayed when assertion is thrown

### Benefits of the assertions

Some of the benefits of using the assertions:

* They are really useful for not having to debug all kind of weird and unexpected errors, especially in runtime. Exact failure point appears when pass by an assertion.
* Developer can be sure that some conditions are met at a given code point. Code turns more reliable.
* Help to detect no so obvious errors happening (affecting performance for example)

### Difference between Assertion and Exception

While assertions are aimed at impossible situations generated from
programming errors, exceptions handle all kind of expected errors and unusual
but logically possible code situations.

Lets review an example: suppose we are writing a math library and created a really fast method to calculate square roots but it only works for positive numbers. Something declared as:

     double sqrt_for_positives(double number)

So what could be an assertion and what an exception for our revolutionary function?

* Exception: if the incoming number is negative (our function only accepts positive numbers), then we will thrown an exception. It was an error by the user but we should consider it a possible scenario since we are offering a public interface.

* Assertion: our square root should never return a negative number. This is not a logical error, it's a completely unexpected error.

## Debugging Gazebo

### Meaningful backtraces

In order to provide meaningful backtraces when using a debugger, such as GDB, Gazebo should be compiled with debugging support enabled. When using the ubuntu packages, specially the ''-dbg'' package, this support is limited but could be enough in most of the situations. This are the three level of traces which can be obtained:

**Maximum level of debugging support**
:This only can be obtained compiling Gazebo from source and setting the `CMAKE_BUILD_TYPE` to `DEBUG`. This will set up no optimizations and debugging symbols. It can be required by developers in situations specially difficult to reproduce.

**Medium level of debugging support**
:This can be obtained installing the ''gazebo-dbg'' package (since 1.4 version) or compiling Gazebo from source using the `RELWITHDEBINFO` `CMAKE_BUILD_TYPE` mode (which is the default if no mode is provided). This will set up ''-O2'' optimization level but provide debugging symbols. This should be the default when firing up gdb to explore errors and submit traces.

**Minimum level of debugging support**
:This one is present in package versions previous to 1.4 (no ''-dbg'' package present) or compiling Gazebo from source using the `RELEASE` `CMAKE_BUILD_TYPE` option. This will set up the maximum level of optimizations and does not provide any debugging symbol information. This traces are particularly difficult to follow.


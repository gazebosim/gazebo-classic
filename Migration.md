### Gazebo 2.0 to 3.0

1. **ConnectionManager::Init**

    Breaks: *ABI*

    A third parameter has been added that specifies the number of timeout iterations. This parameter has a default value of 30.

1. **transport::init**

    Breaks: *ABI*

    A third parameter has been added that specifies the number of timeout iterations. This parameter has a default value of 30.

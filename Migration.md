## Gazebo 2.0 to 3.0

### Modifications

1. **ConnectionManager::Init** `ABI change`

    *Compiliance:* No changes to downstream code required.

    *Note:* A third parameter has been added that specifies the number of timeout iterations. This parameter has a default value of 30.

1. **transport::init** `ABI change`

    *Compiliance:* No changes to downstream code required.

    *Note:* A third parameter has been added that specifies the number of timeout iterations. This parameter has a default value of 30.

### Additions

### Deletions

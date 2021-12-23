# EnvBasedHDMap

## Run Demo

You can run `env_test.py` directly (Need to change the address of the file).



## Dataset

I use INTERACTION dataset. For test, I provide a part of dataset in`Dataset` directory. 



## Map Information

I have transform the information in `.osm` file to `.json` format, where in the directory `Test`. Furthermore, the original map information is in `Map` directory.

There are four `.json` files  for each scene in the `Test` directory.

- `id_node_dict.json`: record the node information in HD map, including  the id, coordinate info of nodes and which way or ways the node owned.

- `id_way_dict.json`: record the way information in HD map, including the id, type info of ways, and what nodes construct this way.

- `id_lane_dict.json`: record the lane information in HD map, including the id info of lanes and their left/right way id.

- `draw_lane_dict.json`: record the lane information which are extract by me. I use these information to draw lanes with `pygame`.

  - `start_str`: the start string of a lane used in highway_env (cf https://github.com/eleurent/highway-env/)

  - `end_str`: the end string of a lane used in highway_env (cf https://github.com/eleurent/highway-env/)

  - `id`: the id, defined by me, to denote the lane

  - `index_coord_dict`: the dictionary, used to record the coordinate of start and end points of a short lane. A lane is made up by many short lanes.

    - For example,

      ```python
      index_coord_dict = {0: [[left way start point, left way end point], [right way start point, right way end point]], 
                          1: [[], []]}
      ```

      

  - `index_type_dict`: the dictionary, used to record the line type (solid or dashed) of a short lane. 

    - For example,

      ```python
      index_type_dict = {0: [left way type, right way type],
                         1: []}
      ```

      

```python
Note that: Comparing to the original .osm file, draw_lane_dict.json may lost some information. Since it is extracted by me to construct origianl scene and is only tested in Merge scene. However, id_node_dict.json, id_way_dict.json and id_lane_dict.json include all information in .osm file.
```








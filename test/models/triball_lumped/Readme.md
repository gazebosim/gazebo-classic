To regenerate the model.sdf from the erb template:
~~~
erb model.sdf.erb > model.sdf
~~~
Note that instantiating this model requires Ruby 2.2 or later
to use the [Vector::cross](http://ruby-doc.org/stdlib-2.2.0/libdoc/matrix/rdoc/Vector.html#method-i-cross)

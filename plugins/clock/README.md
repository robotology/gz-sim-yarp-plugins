### gzyarp::Clock Plugin


| `name`        | `filename`         |
|:-------------:|:------------------:|
| `gzyarp::Clock` |  `gz-sim-yarp-clock-system` |

The `gzyarp::Clock` plugin can be included in world SDF to publish the simulation clock of Gazebo to a `/clock` YARP port. The clock signal published on the `/clock` YARP port can be used by setting the `YARP_CLOCK=/clock` environment variable, to ensure that classes like `yarp::os::PeriodicThread` and functions like `yarp::os::Time::delay` or `yarp::os::Time::now` use the simulated clock instead of the wall clock.

### Usage

Add the `gzyarp::Clock` plugin XML to the top-level world SDF.

> [!IMPORTANT]
> The `gzyarp::Clock` should only be included in the top-level SDF, please do not include it as part of a model that can be inserted in other models or in a world.

An example of its usage can be seen in the [`tutorial/clock/model.sdf`](../../tutorial/clock/model.sdf)

The relevant snippet is:

```xml
    <plugin
      filename="gz-sim-yarp-clock-system"
      name="gzyarp::Clock">
    </plugin>
```

### Reference documentation of `plugin` child XML elements

The `gzyarp::Clock` does not support any `plugin` child XML elements.

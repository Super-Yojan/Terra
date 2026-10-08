# terra-waypoint

One goal in, one body twist out. TerraPhone imports the same follower through `terra-mobile` as `MobileWaypoint`. Zorvane calls this crate when a goal arrives on `terra/rover/<id>/goal`, so the headless run and the phone share the follower.

[API](https://super-yojan.dev/Terra/api/terra_waypoint/index.html) · [source](https://github.com/Super-Yojan/Terra/blob/main/crates/terra-waypoint/src/lib.rs)

## Projection

`GeoOrigin` projects WGS84 into metres: **+x north, +y west**. Latitude must lie in `[-85, 85]` degrees. The scale is the mean metre-per-degree of a 40,075,016.686 m circumference. `from_local` is the inverse.

## Goal wire format

`decode_goal` accepts at most 2048 bytes.

| JSON | Meaning |
| --- | --- |
| `{"cancel": true}` | Drop the latched goal. `cancel` must be the only field. |
| `{"frame":"local","x","y"}` | Metres north and west. Optional `yaw` and `token`. |
| `{"frame":"wgs84","latitude","longitude"}` | Degrees. Optional `yaw` and `token`. |

Unknown fields are rejected. Local coordinates must stay within 20 km of the origin. WGS84 goals use the same latitude and longitude limits as `GeoOrigin`.

`encode_status` writes `WaypointStatus`: `idle`, `active`, or `arrived`, a `goal_id`, optional token, distance, local `x`/`y`, and optional yaw and geographic position.

## Follower defaults

`WaypointConfig` defaults to cruise 1 m/s, yaw gain 1.6, max yaw rate 1.2 rad/s, arrive radius 0.75 m, align yaw 0.12 rad, slow radius 3 m, and heading gate 0.55 rad. Cruise and yaw rate must be at most 5, arrive radius at most 20 m, and the heading gate at most π.

Inside the slow radius the cruise speed scales down, with a floor of 0.2 of cruise. A large heading error holds the linear speed at zero until the vehicle is aligned within the heading gate. `set_goal` on the phone returns false when the origin is missing or the point lies outside `halfExtent`. The phone's waypoint section uses a 49 m half-extent, which matches the Johnson Center tile in Zorvane.

The default phone fields are origin latitude 38.8297, longitude −77.3075, and a goal about 12 m north (38.82981, −77.3075), token `gmu-north`.

# NEXT competition practice

`TERRA_NEXT=1` loads a dedicated practice pitch for the NEXT competition instead of the town or the real-world tile patch. Zatara is the primary rover (spawn slot zero, the existing Terra chassis and differential drive). The scene is a clean approximation of a small-sided soccer field: an 18 m by 12 m pitch, perimeter boards, six tennis balls, and two open-front deposit buckets in the north penalty area. It is not an official field survey, and it does not score a match.

Zatara starts at the origin on the south half, facing the buckets (Bevy −Z). Balls rest around midfield. Drive into a ball to push it; once a ball’s center crosses a bucket mouth, a light pull draws it to the back wall so it stays deposited. The bucket opening is wider than the chassis, so Zatara can nose in and then reverse out. Boards keep balls on the pitch.

## Launch

From the repository:

```sh
cd simulator
TERRA_NEXT=1 TERRA_ZENOH=0 cargo run
```

`TERRA_ZENOH=0` leaves the primary rover on the keyboard. W and S drive forward and back, A and D turn, and Space commands a stop. With Zenoh left on, a remote client drives Zatara as rover 0:

```sh
TERRA_NEXT=1 cargo run
python3 tools/zenoh_client.py --rover 0 drive --linear 0.8 --angular 0 --seconds 3
```

`TERRA_NEXT=1` turns off the practice town and Terrarium tiles and sets the ground to a 32 m square. It also takes precedence over `TERRA_MISSION=1`. `TERRA_ROVER_COUNT` still changes the fleet; extra rovers use the usual spacing around Zatara. `TERRA_HEADLESS=1` runs the same scene without a window.

Startup prints a `Terra NEXT:` line with the ball and bucket counts.

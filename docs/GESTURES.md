# Gestures

## Static gestures

Each finger is classified as extended or folded (see
[ARCHITECTURE.md](ARCHITECTURE.md#gesture-classification)), then matched against
this table. Config names are in `code`.

| Gesture | Config name | Thumb | Index | Middle | Ring | Pinky |
|---|---|:-:|:-:|:-:|:-:|:-:|
| Fist | `fist` | – | – | – | – | – |
| Peace | `peace` | – | ✔ | ✔ | – | – |
| Middle finger | `middle` | any | – | ✔ | – | – |
| One (thumb) | `one` | ✔ | – | – | – | – |
| Two | `two` | ✔ | ✔ | – | – | – |
| Three | `three` | ✔ | ✔ | ✔ | – | – |
| Four | `four` | ✔ | ✔ | ✔ | ✔ | – |
| Open palm | `palm` | ✔ | ✔ | ✔ | ✔ | ✔ |

Any other combination is `none` and interrupts motion detection. Counting
starts from the thumb, as in v1 (`"1"`…`"4"` are accepted as aliases).

Because the tests are relative to the hand, gestures work with the hand tilted
sideways, upside down, close to or far from the camera, and with either hand.

## Motion events

| Event | Config name | How to perform it |
|---|---|---|
| Hold | `hold` | Keep the gesture **still** for ~1 s (a ring fills around your palm) |
| Swipe | `swipe_left`, `swipe_right`, `swipe_up`, `swipe_down` | Move the hand ~15% of the frame within ~0.3 s |
| Flip | `flip` | Turn the hand over (palm ↔ back) while keeping the gesture |

Directions are as seen in the preview window. With `camera.mirror = true`
(default) that matches your own left and right.

## Modes

While a mode is active, motion events are ignored and the hand drives a value.
Show an **open palm** to leave; the mode also ends if your hand is out of view
for 3 s.

| Mode | Enter (default) | Control gesture | Control |
|---|---|---|---|
| Brightness | hold `fist` | `peace` | Raise your hand to brighten, lower it to dim |
| Volume | hold `two` | `three` | Tilt like a dial: lean left = quieter, lean right = louder (±60°) |
| Depth | hold `four` | `one` | Shows how close your hand is (no OS action yet) |

## Mouse mode (`handtracker mouse`)

- The cursor follows your middle knuckle inside the rectangle drawn on the
  preview. The rectangle maps to the whole screen.
- **Pinch** thumb and index to press the left button; separate them to release.
  Hold the pinch and move to drag.
- Losing the hand always releases the button.

## Tips

- Light your hand from the front and avoid a busy background behind it.
- Keep your hand 40–80 cm from the camera; at the frame edge the landmark model
  is less accurate.
- Run with `--dry-run` while learning the gestures.
- If thumbs are misread, adjust `classifier.thumb_extension_ratio` (higher
  values mean the thumb must stick out further to count).

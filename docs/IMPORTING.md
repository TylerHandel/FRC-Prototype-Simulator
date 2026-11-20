# Bringing in your CAD

Three steps in the **Rig CAD** workspace: tell it where the CAD is, wait while it reads it, check
what it found. About five minutes for a whole robot.

## Before the first time: your Onshape API keys

The app reads your CAD through Onshape's API, as you. It needs a pair of keys, made once:

1. Go to **dev-portal.onshape.com** and sign in with your normal Onshape account.
2. Click **API keys**, then **Create new API key**.
3. Tick **Read** (that's all the app needs) and create it.
4. Copy the **access key** and the **secret key**. The secret is shown only once, so paste both
   into the app straight away.

In the app, tick **Remember them on this computer** and you won't be asked again. They're kept in
your user's app data folder, never in a project, so sharing a project never shares your keys.

Each teammate makes their own keys. An import uses about 8 of your account's yearly API calls (an
education account gets about 2,500 a year); the import screen shows how many you've used.

No keys? Export a STEP file from Onshape yourself (right-click the assembly's tab, **Export**,
choose **STEP**) and use **STEP file** instead. You get the same geometry; what you lose is the
bill of materials (so masses come from volumes) and the mates (so joints are marked by clicking).

## 1. Where is your CAD?

1. Open your robot's top-level assembly in Onshape. Copy the address from the browser's address
   bar. It looks like `https://cad.onshape.com/documents/.../w/.../e/...`.
2. In the app: **Rig CAD**, then **Onshape link**. Paste it.
3. Press **Import**.

## 2. Wait

Onshape makes a STEP file of the assembly (a minute or two), then the app reads every part. A
progress bar shows where it's up to. You can keep working in other workspaces.

## 3. Check what it found

The robot appears with each **link** (a set of parts that move together) in its own colour.

1. **Forward and up.** If the robot is lying down or facing the wrong way, change "Forward is the
   CAD's" and "up is" until it stands on the floor with its front (the intake) facing away from you.
2. **Links.** Rename them if you like. If parts are in the wrong link, fix it after the next step:
   click a part, then **Move to link**.
3. **Joints.** Untick any that are wrong.
4. **Rollers.** The app finds wheels and rollers by their names (and the bill of materials), and
   checks each against its round faces. Pick a different wheel for any of them here or later.
5. Press **Use this**.

Then check the motors: the app guesses a motor and a gear ratio for each joint and roller shaft, and
says "guessed" in their names until you change them. Click one in the list to set it.

## Getting the most from your mates

The app makes its joints from your Onshape mates, so the fastest way to a rigged robot is to mate
what really moves:

- a **revolute** mate on each pivot (arm, wrist, intake deploy), with **limits** set;
- a **slider** mate on each elevator stage, with limits;
- **fastened** mates (or a mate group) for everything else.

Parts nobody mated join the nearest link in their sub-assembly; the app tells you how many.

## Another team's CAD

Onshape only shares the mates of your own documents and of fully public ones. For a document that's
shared as a link (export only), the app can still find the joints if the document has the robot in
a second pose (an assembly such as "Scoring Positions"): leave **compare with another assembly**
ticked and it exports that too and compares the two. Joints that didn't move between the two poses
aren't found that way; mark them by clicking.

Please don't share other teams' CAD you've imported; it stays in your project's `cad` folder.

## Marking things by clicking

After **Use this**, click any part in the 3D view:

- **Move to link**: put it (and the rest of its sub-assembly) in another link, or a new one.
- **Roller here**: one of its round faces becomes a roller, on that axis, that size.
- **Joint axis here**: its link turns about that round face (a pivot shaft, a bearing).

## When the CAD changes

**Rig CAD**, **Re-import**. Everything you marked stays attached to its parts: the app finds each
part again by its name, or by its size and place if it was renamed. Anything it can't find is listed
so nothing is lost silently.

# SWARM — VS Code demo

A static presentation demo with three simulated UAV deliveries in Columbia, SC. Each store prepares the package for 30 seconds before its UAV takes off. This project does not receive live vehicle telemetry.

## Start with VS Code Live Server

1. Extract the ZIP and open the **flightpath-vscode** folder using **File → Open Folder** in Visual Studio Code.
2. In the Extensions view (`Ctrl+Shift+X` on Windows), install **Live Server** by Ritwick Dey (`ritwickdey.LiveServer`). Accept the workspace recommendation if VS Code shows it.
3. Open `index.html` and click **Go Live** in the VS Code status bar, or right-click it and choose **Open with Live Server**. A browser should open at `http://127.0.0.1:5500/` (the address may show `localhost` instead).
4. Select a demo delivery in the page or enter one of the numbers below and click **Track**. Use Pause, Replay, and speed controls for the presentation.
5. Edit and **save** `index.html`, `css/styles.css`, or `js/app.js`; Live Server refreshes the page in your browser. Click the status-bar port to stop the server.

## Files

- `index.html` — page structure and external map/font links
- `css/styles.css` — layout and appearance
- `js/app.js` — route animation, ETA, tracking form, and map tile layout
- `.vscode/` — Live Server recommendation and local port settings

## Demo numbers

| Tracking number | Store | Destination |
| --- | --- | --- |
| UAV-COLA-2047 | Publix · Gervais Place | Finlay Park |
| UAV-COLA-3186 | Mast General Store | Riverfront Park |
| UAV-COLA-5729 | Publix · Gervais Place | Columbia Museum of Art |

The map uses OpenStreetMap tiles and Google Fonts over the internet, so an internet connection is needed for full map display. Tile positioning and route drawing use the included CSS and JavaScript; no third-party map layout files are required. Every coordinate and arrival estimate is simulated.

"""Build a compact recorded 2047 replay from a passing owned SITL run."""
import argparse
import json
from pathlib import Path

from .geographic_flight import CATALOG


def build(run_dir, output):
    run_dir, output = Path(run_dir), Path(output)
    result = json.loads((run_dir/"result.json").read_text())
    expected = CATALOG["UAV-COLA-2047"]["path"]
    if result.get("status") != "PASS" or result.get("payload") != "retained":
        raise ValueError("archive source must be a passing, retained-payload run")
    if [list(p) for p in result["route"]["path"]] != expected:
        raise ValueError("archive source must be the full UAV-COLA-2047 route")
    latest = {}
    frames = []
    stage = "PREPARING"
    last_wall = float("-inf")
    target_lat, target_lon = expected[-1]
    for line in (run_dir/"telemetry.jsonl").open():
        item = json.loads(line)
        kind = item.get("mavpackettype")
        if kind:
            latest[kind] = item
        if kind != "GLOBAL_POSITION_INT" or item["wall"]-last_wall < .45:
            continue
        if not all(x in latest for x in ("SYS_STATUS","GPS_RAW_INT","ATTITUDE","HEARTBEAT")):
            continue
        extended = latest.get("EXTENDED_SYS_STATE")
        near_target = abs(item["lat"]/1e7-target_lat)<.00003 and abs(item["lon"]/1e7-target_lon)<.00003
        if stage == "PREPARING" and item["relative_alt"]>3000:
            stage = "OUTBOUND"
        if stage == "OUTBOUND" and near_target and item["relative_alt"]<500 and extended and extended.get("landed_state")==1:
            stage = "TARGET_LANDED"
        if stage == "TARGET_LANDED" and item["relative_alt"]>3000:
            stage = "RETURNING"
        frames.append(dict(t=item["wall"],position=item,battery=latest["SYS_STATUS"],
                           gps=latest["GPS_RAW_INT"],attitude=latest["ATTITUDE"],
                           heartbeat=latest["HEARTBEAT"],extended=extended,
                           mission=latest.get("MISSION_CURRENT"),status=latest.get("STATUSTEXT"),phase=stage))
        last_wall=item["wall"]
    if len(frames)<20 or not {'OUTBOUND','TARGET_LANDED','RETURNING'}.issubset({f['phase'] for f in frames}):
        raise ValueError("recording lacks verified flight, target stop and return phases")
    archive=dict(label="Recorded full-distance 2047 target-landing SITL round trip",
                 route=result["route"],result=result,frames=frames)
    output.write_text(json.dumps(archive,separators=(',',':'))+"\n")
    return len(frames)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir",type=Path)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    print(f"Archived {build(args.run_dir,args.output)} SITL frames")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""push.py <local_file> [container_dest]  --  copy a file to the rover container.

ssh.py runs a single command string with NO stdin forwarding (paramiko exec_command
is given no input), so `python3 ssh.py "cat > /tmp/x" < file` silently writes an EMPTY
file -- it looks like it worked and it does not. This helper uses SFTP for the host hop
and `docker cp` for the container hop, then prints both md5s so the copy is verified
rather than assumed.

Default destination is /root/<basename> inside rover_ros2.
"""
import hashlib
import os
import sys

import paramiko

HOST, USER, PASS = "10.7.12.41", "nithin", "6238764749"


def main():
    if len(sys.argv) < 2:
        print("usage: push.py <local_file> [container_dest]", file=sys.stderr)
        return 2
    local = sys.argv[1]
    dest = sys.argv[2] if len(sys.argv) > 2 else "/root/" + os.path.basename(local)
    staging = "/tmp/" + os.path.basename(local)

    with open(local, "rb") as f:
        local_md5 = hashlib.md5(f.read()).hexdigest()

    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(HOST, username=USER, password=PASS, timeout=15)

    sftp = cli.open_sftp()
    sftp.put(local, staging)
    sftp.close()

    # `docker cp` copies the SOURCE file's mode. Pushing a stack.sh that was 644 in the
    # repo silently stripped +x off /root/stack.sh, and every `~/rover <anything>` then
    # died with "OCI runtime exec failed: ... permission denied" -- a deploy that
    # verified its own md5 perfectly while leaving the rover unusable. Restore the bit
    # explicitly for anything that is executed rather than interpreted.
    chmod = " && docker exec rover_ros2 chmod +x " + dest if local.endswith(".sh") else ""
    cmd = (f"docker cp {staging} rover_ros2:{dest}{chmod} "
           f"&& docker exec rover_ros2 md5sum {dest}")
    _, stdout, stderr = cli.exec_command(cmd, timeout=60)
    out = stdout.read().decode(errors="replace").strip()
    err = stderr.read().decode(errors="replace").strip()
    rc = stdout.channel.recv_exit_status()
    cli.close()

    if rc != 0:
        print(f"push FAILED rc={rc}: {err}", file=sys.stderr)
        return rc

    remote_md5 = out.split()[0] if out else ""
    ok = remote_md5 == local_md5
    print(f"{'OK  ' if ok else 'MISMATCH'} {local} -> rover_ros2:{dest}")
    print(f"  local  {local_md5}")
    print(f"  remote {remote_md5}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

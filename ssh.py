#!/usr/bin/env python3
"""Quick SSH runner for the Pi5 rover. Usage: python3 ssh.py <command...>"""
import sys
import paramiko
HOST, USER, PASS = "10.7.12.41", "nithin", "6238764749"
cmd = " ".join(sys.argv[1:]) if len(sys.argv) > 1 else "echo connected"
cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect(HOST, username=USER, password=PASS, timeout=15)
stdin, stdout, stderr = cli.exec_command(cmd, timeout=60)
out = stdout.read().decode(errors="replace"); err = stderr.read().decode(errors="replace")
rc = stdout.channel.recv_exit_status(); cli.close()
if out: sys.stdout.write(out)
if err: sys.stderr.write(err)
sys.exit(rc)

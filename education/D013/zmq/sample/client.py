#!/usr/bin/env python3
"""client"""

import zmq


def main():
    """client"""
    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.connect("tcp://localhost:5556")

    topic = "Jerry"
    sub.setsockopt_string(zmq.SUBSCRIBE, topic)
    print(f"[CLIENT] connected. subscribed: {topic}")

    try:
        while True:
            _topic, _msg = sub.recv_multipart()
            topic = _topic.decode()
            msg = _msg.decode(errors="replace")
            print(f"[{topic}] {msg}")
    except KeyboardInterrupt:
        pass
    finally:
        sub.close(0)
        ctx.term()
        print("\n[CLIENT] bye")


if __name__ == "__main__":
    main()

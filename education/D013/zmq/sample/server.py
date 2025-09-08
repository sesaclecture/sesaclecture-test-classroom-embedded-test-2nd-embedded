#!/usr/bin/env python3
"""test"""

import time

import zmq


def main():
    """main"""
    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.bind("tcp://localhost:5556")
    time.sleep(0.5)

    print("[SERVER] PUB bind at tcp://localhost:5556")

    try:
        topic = "Jerry"
        msg = "Hello World!"
        pub.send_multipart([topic.encode(), msg.encode()])
        print(f"[SERVER] Sent: topic->{topic} msg->{msg}")
    except KeyboardInterrupt:
        pass
    finally:
        pub.close(0)
        ctx.term()
        print("[server] Complete!")


if __name__ == "__main__":
    main()

import transport_zmq


def main():
    print("bla")
    trans=transport_zmq.TransportZmq(b"tcp://127.0.0.1:2000",3)

if __name__ == "__main__":
    # execute only if run as a script
    main()


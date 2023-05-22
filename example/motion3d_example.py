#!/usr/bin/env python3
import motion3d as m3d


def main():
    dq = m3d.DualQuaternionTransform()
    print(dq.toString())


if __name__ == '__main__':
    main()

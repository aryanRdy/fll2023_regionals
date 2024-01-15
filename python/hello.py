from hub import light_matrix
import runloop
import motorpair


async def main():
    # write your code here
    await light_matrix.write('Hello, World!')
    mp = motorpair.MotorPair('A', 'B')

runloop.run(main())

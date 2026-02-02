import asyncio
import json
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from agent.agent import agent
from agents import Runner

async def debug_agent():
    print("Testing agent with debug output...")

    result = await Runner.run(
        starting_agent=agent,
        input="What is Humanoid Robotics?"
    )

    print("Full result object:")
    print(f"Type: {type(result)}")
    print(f"Dir: {dir(result)}")

    print(f"\nResult attributes:")
    for attr in dir(result):
        if not attr.startswith('_'):
            try:
                value = getattr(result, attr)
                print(f"  {attr}: {type(value)} = {str(value)[:200]}...")
            except Exception as e:
                print(f"  {attr}: <Error accessing - {str(e)}>")

    if hasattr(result, 'turns') and result.turns:
        print(f"\nTurns found: {len(result.turns)}")
        for i, turn in enumerate(result.turns):
            print(f"  Turn {i}: {type(turn)}")
            print(f"    Dir: {dir(turn)}")

            if hasattr(turn, 'tool_calls'):
                print(f"    Tool calls: {len(turn.tool_calls) if turn.tool_calls else 0}")
                for j, tool_call in enumerate(turn.tool_calls or []):
                    print(f"      Tool call {j}: {type(tool_call)}")
                    print(f"        Name: {getattr(tool_call, 'name', 'N/A')}")
                    print(f"        Result: {getattr(tool_call, 'result', 'N/A')}")
                    print(f"        Result type: {type(getattr(tool_call, 'result', None))}")

    print(f"\nFinal output: {result.final_output[:500]}...")

if __name__ == "__main__":
    asyncio.run(debug_agent())
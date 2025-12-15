import asyncio
import json
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from agent.agent import agent
from agents import Runner

async def debug_agent_detailed():
    print("Testing agent with detailed debug output...")

    result = await Runner.run(
        starting_agent=agent,
        input="What is Humanoid Robotics?"
    )

    print("=== DETAILED RESULT DEBUG ===")
    print(f"Result type: {type(result)}")

    # Check new_items structure
    print(f"\n--- new_items ---")
    if hasattr(result, 'new_items'):
        print(f"new_items type: {type(result.new_items)}")
        print(f"new_items length: {len(result.new_items) if result.new_items else 0}")
        for i, item in enumerate(result.new_items or []):
            print(f"  Item {i}: {type(item)}")
            print(f"    dir: {[attr for attr in dir(item) if not attr.startswith('_')]}")
            if hasattr(item, '__dict__'):
                print(f"    __dict__: {item.__dict__}")
            if hasattr(item, 'result'):
                print(f"    result: {type(item.result)} = {item.result}")
            if hasattr(item, 'agent'):
                print(f"    agent: {type(item.agent)}")

    # Check raw_responses structure
    print(f"\n--- raw_responses ---")
    if hasattr(result, 'raw_responses'):
        print(f"raw_responses type: {type(result.raw_responses)}")
        print(f"raw_responses length: {len(result.raw_responses) if result.raw_responses else 0}")
        for i, response in enumerate(result.raw_responses or []):
            print(f"  Response {i}: {type(response)}")
            if hasattr(response, 'output'):
                print(f"    output type: {type(response.output)}")
                for j, output_item in enumerate(response.output or []):
                    print(f"      Output item {j}: {type(output_item)}")
                    print(f"        dir: {[attr for attr in dir(output_item) if not attr.startswith('_')]}")
                    if hasattr(output_item, 'name'):
                        print(f"        name: {output_item.name}")
                    if hasattr(output_item, 'result'):
                        print(f"        result: {type(output_item.result)} = {output_item.result}")

    print(f"\nFinal output preview: {result.final_output[:200]}...")

if __name__ == "__main__":
    asyncio.run(debug_agent_detailed())
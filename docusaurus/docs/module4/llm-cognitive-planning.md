---
doc_id: module4_m4_ch04
title: "Cognitive Planning with LLMs"
module: "Module 4: Vision-Language-Action (VLA)"
estimated_tokens: 1200
embedding_required: true
chunk_hint_tokens: 500
---

# Cognitive Planning with LLMs

## Objective

This chapter introduces students to the use of Large Language Models (LLMs) for cognitive planning in robotics. Students will learn how LLMs can generate complex action sequences from high-level goals, enabling robots to reason about tasks and plan appropriate behaviors in dynamic environments.

## Learning Outcomes

After completing this chapter, students will be able to:
1. Understand the role of LLMs in cognitive robotics and planning
2. Design prompt strategies for generating robot action plans with LLMs
3. Implement LLM-based planning systems that integrate with robotics frameworks
4. Evaluate and validate LLM-generated plans for safety and feasibility

## Theory

Cognitive planning in robotics involves generating sequences of actions that achieve specified goals while considering constraints, resources, and environmental conditions. Traditional planning approaches rely on formal logical representations and search algorithms, but Large Language Models (LLMs) offer an alternative approach based on learned patterns of reasoning from vast amounts of text data.

### LLMs as Cognitive Planners

LLMs can function as cognitive planners by leveraging their learned understanding of relationships between goals, actions, and outcomes. When presented with a goal and environmental context, LLMs can generate plausible action sequences to achieve the goal, drawing on their knowledge of how tasks are typically accomplished.

Key characteristics of LLM-based planning:

1. **Common-sense reasoning**: LLMs incorporate common-sense knowledge that can inform planning decisions
2. **Natural language interface**: Goals can be specified in natural language, making planning more accessible
3. **Flexibility**: LLMs can adapt to novel situations not explicitly covered in traditional planners
4. **Learning from text**: LLMs can incorporate planning knowledge from instructions, procedures, and descriptions in text

### Integration with Robotics Systems

LLMs interface with robotics systems through several pathways:

1. **Goal specification**: Natural language goals are processed by LLMs to generate action plans
2. **Environmental context**: Sensor data and environmental information are converted to natural language for LLM processing
3. **Action translation**: LLM-generated plans are translated to robot executable actions
4. **Feedback integration**: Execution outcomes are fed back to refine the LLM's understanding

### Planning Approaches with LLMs

Several approaches exist for using LLMs in planning:

1. **Direct generation**: The LLM generates action sequences directly from goals
2. **Chain-of-thought reasoning**: The LLM reasons step-by-step before generating actions
3. **Tool integration**: The LLM calls external tools (simulators, sensors) during planning
4. **Iterative refinement**: The LLM refines plans based on feedback and environmental changes

### Challenges in LLM-Based Planning

Despite their potential, LLMs face several challenges in robotics planning:

1. **Reliability**: LLMs can generate factually incorrect or unsafe plans
2. **Grounding**: Connecting abstract LLM knowledge to specific robot capabilities
3. **Real-time constraints**: Planning may not meet real-time requirements for robotics
4. **Validation**: Ensuring LLM-generated plans are safe and effective in the real world

### Safety and Validation Mechanisms

Safety is paramount in LLM-based planning for robots. Key mechanisms include:

1. **Plan validation**: Checking plans for safety and feasibility before execution
2. **Human oversight**: Implementing approval processes for LLM-generated plans
3. **Execution monitoring**: Continuously monitoring plan execution and intervening if needed
4. **Fallback mechanisms**: Having alternative plans when LLM plans fail

## Practical Examples

### Example 1: Basic LLM Planning Interface

Implementing a basic interface for LLM-based cognitive planning:

```python
import openai
import json
from typing import List, Dict, Any

class LLMPlanner:
    def __init__(self, api_key: str, model: str = "gpt-3.5-turbo"):
        """
        Initialize LLM-based planning system
        """
        openai.api_key = api_key
        self.model = model
        self.system_prompt = """
        You are an expert robot cognitive planner. Given a robot goal and environmental context, 
        generate a sequence of high-level actions to achieve the goal. Each action should be specific 
        enough for a robot to execute. Return the plan as a JSON array of action dictionaries.
        
        Each action dictionary should have:
        - 'action': The name of the action (e.g., 'navigate_to', 'grasp_object', 'manipulate_object')
        - 'parameters': A dictionary of parameters needed for the action
        - 'description': A human-readable description of the action
        """
    
    def generate_plan(self, goal: str, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Generate a plan for the given goal and context
        """
        user_message = f"""
        Goal: {goal}
        
        Environmental Context:
        {json.dumps(context, indent=2)}
        
        Please generate a step-by-step plan to achieve this goal.
        """
        
        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": user_message}
                ],
                temperature=0.3,  # Lower temperature for more consistent planning
                max_tokens=1000
            )
            
            plan_text = response.choices[0].message['content'].strip()
            
            # Extract JSON from response
            json_start = plan_text.find('[')
            json_end = plan_text.rfind(']') + 1
            
            if json_start != -1 and json_end != 0:
                plan_json = plan_text[json_start:json_end]
                plan = json.loads(plan_json)
                
                return {
                    'status': 'success',
                    'plan': plan,
                    'raw_response': plan_text,
                    'model_used': self.model
                }
            else:
                # If no JSON found, return the raw response
                return {
                    'status': 'error',
                    'error': 'Could not extract valid JSON plan from response',
                    'raw_response': plan_text
                }
        
        except Exception as e:
            return {
                'status': 'error',
                'error': str(e),
                'raw_response': None
            }

    def validate_plan(self, plan: List[Dict]) -> Dict[str, Any]:
        """
        Basic validation for generated plans
        """
        validation_result = {
            'is_valid': True,
            'issues': [],
            'safe': True,
            'warnings': []
        }
        
        # Check if plan is a list
        if not isinstance(plan, list):
            validation_result['is_valid'] = False
            validation_result['issues'].append("Plan is not a list of actions")
            return validation_result
        
        # Check each action in the plan
        for i, action in enumerate(plan):
            if not isinstance(action, dict):
                validation_result['is_valid'] = False
                validation_result['issues'].append(f"Action {i} is not a dictionary")
                continue
            
            # Check required keys
            required_keys = ['action', 'parameters', 'description']
            for key in required_keys:
                if key not in action:
                    validation_result['is_valid'] = False
                    validation_result['issues'].append(f"Action {i} missing required key: {key}")
            
            # Check for potentially unsafe actions
            unsafe_keywords = ['damage', 'break', 'harm', 'destroy', 'injure']
            action_desc = action.get('description', '').lower()
            for keyword in unsafe_keywords:
                if keyword in action_desc:
                    validation_result['safe'] = False
                    validation_result['warnings'].append(f"Action {i} contains potentially unsafe language: {keyword}")
        
        return validation_result

# Usage example
def example_basic_planning():
    # Initialize planner (replace with your actual API key)
    planner = LLMPlanner(api_key="your-openai-api-key")
    
    # Define goal and context
    goal = "Pick up the red cup from the kitchen and bring it to the living room"
    context = {
        "robot_position": "starting room",
        "known_objects": [
            {"name": "red cup", "position": "kitchen counter", "color": "red"},
            {"name": "blue book", "position": "table", "color": "blue"}
        ],
        "known_locations": ["kitchen", "living room", "bedroom", "hallway"],
        "robot_capabilities": ["navigate", "grasp", "manipulate", "transport"]
    }
    
    # Generate plan
    result = planner.generate_plan(goal, context)
    
    if result['status'] == 'success':
        print("Generated Plan:")
        for i, action in enumerate(result['plan']):
            print(f"{i+1}. {action['description']}")
        
        # Validate plan
        validation = planner.validate_plan(result['plan'])
        print(f"\nValidation: {validation}")
    else:
        print(f"Planning failed: {result['error']}")
```

### Example 2: LLM with Environmental Feedback Loop

Implementing a more sophisticated planning system with feedback:

```python
class FeedbackLLMPlanner:
    def __init__(self, api_key: str, model: str = "gpt-3.5-turbo"):
        """
        LLM-based planner with environmental feedback
        """
        openai.api_key = api_key
        self.model = model
        self.system_prompt = """
        You are an expert robot cognitive planner. Your task is to generate action plans 
        that adapt to environmental feedback. Consider the following principles:
        
        1. Plans should be flexible and adaptable
        2. Consider potential obstacles and alternative paths
        3. Prioritize safety in all actions
        4. Break complex tasks into manageable subtasks
        
        Return plans as JSON arrays with action, parameters, and description.
        """
    
    def generate_adaptive_plan(self, goal: str, initial_context: Dict, 
                              max_iterations: int = 5) -> Dict[str, Any]:
        """
        Generate plan with ability to adapt based on feedback
        """
        context = initial_context.copy()
        full_plan = []
        iteration = 0
        
        while iteration < max_iterations:
            # Generate next part of plan based on current context
            plan_part = self._generate_plan_part(goal, context)
            
            if plan_part['status'] != 'success':
                return {
                    'status': 'error',
                    'error': f"Failed at iteration {iteration}: {plan_part['error']}",
                    'partial_plan': full_plan
                }
            
            # Add to full plan
            full_plan.extend(plan_part['plan'])
            
            # Simulate execution feedback (in a real system, this would come from the robot)
            feedback = self._simulate_feedback(plan_part['plan'], context)
            
            # Update context based on feedback
            context.update(feedback.get('updated_context', {}))
            
            # Check if goal is achieved
            if feedback.get('goal_achieved', False):
                return {
                    'status': 'success',
                    'plan': full_plan,
                    'final_context': context,
                    'iterations': iteration + 1
                }
            
            # Check if we need to replan
            if feedback.get('replan_needed', False):
                print(f"Replanning needed at iteration {iteration + 1}")
                continue
            
            iteration += 1
        
        return {
            'status': 'partial',
            'plan': full_plan,
            'final_context': context,
            'iterations': iteration,
            'message': 'Max iterations reached without achieving goal'
        }
    
    def _generate_plan_part(self, goal: str, context: Dict) -> Dict[str, Any]:
        """
        Generate part of the plan based on current context
        """
        user_message = f"""
        Goal: {goal}
        
        Current Context:
        {json.dumps(context, indent=2)}
        
        Generate the next 3-5 actions to progress toward the goal, 
        considering the current situation and any obstacles mentioned.
        """
        
        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": user_message}
                ],
                temperature=0.3,
                max_tokens=800
            )
            
            plan_text = response.choices[0].message['content'].strip()
            
            # Extract JSON from response
            json_start = plan_text.find('[')
            json_end = plan_text.rfind(']') + 1
            
            if json_start != -1 and json_end != 0:
                plan_json = plan_text[json_start:json_end]
                plan = json.loads(plan_json)
                
                return {
                    'status': 'success',
                    'plan': plan,
                    'raw_response': plan_text
                }
            else:
                return {
                    'status': 'error',
                    'error': 'Could not extract valid JSON plan from response',
                    'raw_response': plan_text
                }
        
        except Exception as e:
            return {
                'status': 'error',
                'error': str(e),
                'raw_response': None
            }
    
    def _simulate_feedback(self, plan_executed: List[Dict], context: Dict) -> Dict[str, Any]:
        """
        Simulate environmental feedback (in real implementation, this comes from the robot)
        """
        # This is a simplified simulation of what feedback might look like
        # In a real system, this would come from sensor data and action execution results
        
        # Simulate a random chance of obstacle being detected
        import random
        
        feedback = {
            'updated_context': {},
            'replan_needed': False,
            'goal_achieved': False
        }
        
        # For this simulation, let's say there's a 20% chance of needing to replan
        if random.random() < 0.2:
            feedback['replan_needed'] = True
            feedback['updated_context'] = {
                'obstacle_detected': True,
                'obstacle_location': 'kitchen entrance',
                'available_detour': 'through hallway'
            }
        
        # For this simulation, let's say goal is achieved when the plan includes "task completed"
        for action in plan_executed:
            if "completed" in action.get('description', '').lower():
                feedback['goal_achieved'] = True
                break
        
        return feedback

# Usage example with feedback loop
def example_feedback_planning():
    planner = FeedbackLLMPlanner(api_key="your-openai-api-key")
    
    goal = "Navigate to the kitchen, pick up the red cup, and return to the starting position"
    initial_context = {
        "robot_position": "starting room",
        "known_objects": [
            {"name": "red cup", "position": "kitchen counter", "color": "red", "graspable": True}
        ],
        "known_locations": ["kitchen", "starting room", "hallway"],
        "robot_capabilities": ["navigate", "grasp", "manipulate", "transport"]
    }
    
    result = planner.generate_adaptive_plan(goal, initial_context)
    
    print(f"Planning Result: {result['status']}")
    if result['status'] == 'success':
        print(f"Plan completed in {result['iterations']} iterations")
        print("Final Plan:")
        for i, action in enumerate(result['plan']):
            print(f"{i+1}. {action['description']}")
    elif result['status'] == 'partial':
        print(f"Plan partially completed after {result['iterations']} iterations")
    else:
        print(f"Planning failed: {result.get('error', 'Unknown error')}")
```

## Hands-on Lab

### Prerequisites
- Understanding of LLM APIs and prompting
- Basic knowledge of robotics concepts
- Python programming skills

### Step 1: Set Up LLM Planning Environment
1. Obtain an API key for an LLM service (e.g., OpenAI, Anthropic)
2. Install required Python packages (`openai`, `tiktoken`)
3. Test basic LLM functionality to ensure it's working correctly

### Step 2: Implement Basic Plan Generation
1. Create the basic LLM planner from Example 1
2. Test with simple robot goals like "Navigate to the kitchen"
3. Evaluate the quality and structure of generated plans

### Step 3: Add Validation Mechanisms
1. Implement the plan validation system
2. Test with both valid and potentially unsafe plans
3. Refine the validation rules based on your requirements

### Step 4: Implement Adaptive Planning
1. Create the feedback-aware planner from Example 2
2. Simulate various environmental conditions and obstacles
3. Observe how the planner adapts to changing conditions

### Step 5: Integrate with Robotics Framework
1. Connect the planner to a simple robotics simulation
2. Test the end-to-end process: goal → LLM plan → simulated execution
3. Evaluate the effectiveness of LLM-generated plans in achieving goals

## Exercises

1. Implement a more sophisticated plan validation system that checks for physical feasibility. How would you verify that a plan like "grasp the object above your head" is achievable by a specific robot model?

2. Research and implement a prompting technique that improves the reliability of LLM planning for robotics. How can you structure prompts to reduce hallucinations and increase plan safety?

3. Design a system that learns from plan execution outcomes to improve future planning. How would you incorporate feedback about successful and failed plans?

4. Compare the computational requirements of LLM-based planning versus traditional symbolic planning methods. What are the trade-offs in terms of time, accuracy, and adaptability?

5. Create a hybrid planning system that combines LLM high-level reasoning with traditional path planning for navigation tasks. How would you integrate these approaches?

## Summary

This chapter explored how Large Language Models can serve as cognitive planners for robotics, generating action sequences from high-level goals. We implemented practical examples showing basic plan generation and adaptive planning with environmental feedback. The integration of LLMs into the VLA paradigm enables more flexible and natural interaction between humans and robots, allowing for complex tasks to be specified in natural language. However, careful attention must be paid to safety, validation, and reliability when using LLMs for robot planning.

## Further Reading

- "Language Models for Robotics: A Review of Large Language Models in Robotic Control" - Research Survey
- "Cognitive Robotics: Integrating Planning and Learning" - Academic Book Chapter
- "Large Language Models for Robotics: Opportunities and Challenges" - Conference Paper
- "Safe and Grounded Language-Conditioned Planning for Human-Robot Interaction" - Technical Paper
- "Neural-Symbolic Integration for Robotic Planning" - Research Article
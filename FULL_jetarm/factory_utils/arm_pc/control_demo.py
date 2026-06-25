import action_group_controller as controller

# 动作组需要保存在当前路径的ActionGroups下(the action group needs to be saved in 'ActionGroups' under the current path)
controller.runAction('test') # 参数为动作组的名称，不包含后缀，以字符形式传入(The parameter is the name of the action group, without suffix, passed in as a string)

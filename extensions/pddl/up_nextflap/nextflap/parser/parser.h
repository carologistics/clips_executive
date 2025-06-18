#ifndef PARSER_H
#define PARSER_H

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Parses the domain and problem files. Stores the      */
/* result in a variable of type ParsedTask.             */
/********************************************************/

#include "parsedTask.h"
#include "syntaxAnalyzer.h"

class Parser {
private:
  SyntaxAnalyzer *syn;
  ParsedTask *task;
  void parseRequirements();
  void parseTypes();
  void parseParentTypes(std::vector<unsigned int> &types, bool allowNumber);
  void parseConstants();
  void parsePredicates();
  Function parsePredicate();
  void parseVariableList(std::vector<Variable> &parameters);
  void parseControlVariableList(std::vector<Variable> &parameters);
  void parseFunctions();
  void parseDurativeAction();
  void parseDurationConstraint(std::vector<Duration> &duration,
                               const std::vector<Variable> &parameters,
                               const std::vector<Variable> &controlVars);
  NumericExpression
  parseNumericExpression(const std::vector<Variable> &parameters,
                         const std::vector<Variable> &controlVars);
  unsigned int parseFunctionHead(std::vector<Term> &fncParams,
                                 const std::vector<Variable> &parameters);
  Term parseTerm(const std::vector<unsigned int> &validTypes,
                 const std::vector<Variable> &parameters,
                 const std::vector<Variable> &controlVars);
  Term parseTerm(const std::vector<Variable> &parameters,
                 const std::vector<Variable> &controlVars);
  DurativeCondition
  parseDurativeCondition(const std::vector<Variable> &parameters,
                         const std::vector<Variable> &controlVars);
  void parseGoalDescription(GoalDescription &goal,
                            const std::vector<Variable> &parameters,
                            const std::vector<Variable> &controlVars);
  Literal parseLiteral(const std::vector<Variable> &parameters);
  void mergeVariables(std::vector<Variable> &mergedParameters,
                      const std::vector<Variable> &params1,
                      const std::vector<Variable> &params2);
  void parseADLGoalDescription(GoalDescription &goal,
                               const std::vector<Variable> &parameters,
                               const std::vector<Variable> &controlVars);
  void parseGoalDescriptionComparison(GoalDescription &goal,
                                      const std::vector<Variable> &parameters,
                                      const std::vector<Variable> &controlVars);
  DurativeEffect parseDurativeEffect(const std::vector<Variable> &parameters,
                                     const std::vector<Variable> &controlVars);
  void parseTimedEffect(TimedEffect &timedEffect,
                        const std::vector<Variable> &parameters,
                        const std::vector<Variable> &controlVars);
  ContinuousEffect
  parseContinuousEffect(const std::vector<Variable> &parameters,
                        const std::vector<Variable> &controlVars);
  AssignmentContinuousEffect
  parseAssignmentContinuousEffect(const std::vector<Variable> &parameters,
                                  const std::vector<Variable> &controlVars);
  FluentAssignment
  parseFluentAssignment(const std::vector<Variable> &parameters,
                        const std::vector<Variable> &controlVars);
  EffectExpression
  parseEffectExpression(const std::vector<Variable> &parameters,
                        const std::vector<Variable> &controlVars,
                        bool isAssign);
  void parseEffectOperation(EffectExpression &exp,
                            const std::vector<Variable> &parameters,
                            const std::vector<Variable> &controlVars,
                            bool isAssign);
  void parseAction();
  void parseConstraints();
  Constraint parseConstraint(const std::vector<Variable> &parameters,
                             const std::vector<Variable> &controlVars);
  void parseConstraintGoal(Constraint &constraint,
                           const std::vector<Variable> &parameters,
                           const std::vector<Variable> &controlVars);
  void parseDerivedPredicates();
  Precondition parsePrecondition(const std::vector<Variable> &parameters,
                                 const std::vector<Variable> &controlVars);
  Effect parseEffect(const std::vector<Variable> &parameters,
                     const std::vector<Variable> &controlVars);
  void parseObjects();
  void parseInit();
  Fact parseFact();
  void parseGoal();
  void parseMetric();
  Metric parseMetricExpression();
  unsigned int parseFluent(std::vector<unsigned int> &parameters);
  void parseLength();

public:
  Parser();
  ~Parser();
  ParsedTask *parseDomain(char *domainFileName);
  ParsedTask *parseProblem(char *problemFileName);
};

#endif

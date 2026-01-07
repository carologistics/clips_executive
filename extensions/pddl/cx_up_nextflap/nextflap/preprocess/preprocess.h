#ifndef PREPROCESS_H
#define PREPROCESS_H

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Removes complex features from the ParsedTask, such   */
/* as universal and existential quantifiers,            */
/* disjunctions, etc.                                   */
/********************************************************/

#include "../parser/parsedTask.h"
#include "preprocessedTask.h"

struct FeatureList {
  int universalQuantifierPrec;
  int existentialQuantifierPrec;
  int implicationPrec;
  int disjunctionPrec;
  int universalQuantifierEff;
  int existentialQuantifierEff;
  int implicationEff;
  int disjunctionEff;
  int conditionalEff;
};

class Preprocess {
private:
  ParsedTask *task;
  PreprocessedTask *prepTask;
  void preprocessOperators();
  void checkPreconditionFeatures(Precondition &prec, FeatureList *features);
  void checkPreconditionFeatures(DurativeCondition &prec,
                                 FeatureList *features);
  void checkGoalFeatures(GoalDescription &goal, FeatureList *features,
                         bool prec);
  void checkGoalFeatures(DurativeCondition &goal, FeatureList *features,
                         bool prec);
  void checkEffectFeatures(Effect &eff, FeatureList *features);
  void checkEffectFeatures(DurativeEffect &eff, FeatureList *features);
  void preprocessAction(Action a, FeatureList *features, bool isTIL,
                        bool isGoal);
  void preprocessAction(DurativeAction a, FeatureList *features, bool isTIL,
                        bool isGoal);
  void removeQuantifiers(Precondition &prec, unsigned int numParameters);
  void removeQuantifiers(DurativeCondition &prec, unsigned int numParameters);
  void removeQuantifiers(Effect &eff, unsigned int numParameters);
  void removeQuantifiers(DurativeEffect &eff, unsigned int numParameters);
  void removeQuantifiers(GoalDescription &goal, unsigned int numParameters);
  void replaceQuantifierParameter(Precondition &prec, Precondition term,
                                  unsigned int paramNumber,
                                  unsigned int numParameters);
  void replaceQuantifierParameter(DurativeCondition &prec,
                                  DurativeCondition term,
                                  unsigned int paramNumber,
                                  unsigned int numParameters);
  void replaceQuantifierParameter(Effect &eff, Effect term,
                                  unsigned int paramNumber,
                                  unsigned int numParameters);
  void replaceQuantifierParameter(DurativeEffect &eff, DurativeEffect term,
                                  unsigned int paramNumber,
                                  unsigned int numParameters);
  void replaceQuantifierParameter(GoalDescription &goal, GoalDescription term,
                                  unsigned int paramNumber,
                                  unsigned int numParameters);
  void replaceParameter(Precondition &term, unsigned int paramToReplace,
                        unsigned int objectIndex);
  void replaceParameter(DurativeCondition &term, unsigned int paramToReplace,
                        unsigned int objectIndex);
  void replaceParameter(Literal &literal, unsigned int paramToReplace,
                        unsigned int objectIndex);
  void replaceParameter(GoalDescription &goal, unsigned int paramToReplace,
                        unsigned int objectIndex);
  void replaceParameter(Term &term, unsigned int paramToReplace,
                        unsigned int objectIndex);
  void replaceParameter(NumericExpression &exp, unsigned int paramToReplace,
                        unsigned int objectIndex);
  void replaceParameter(FluentAssignment &fa, unsigned int paramToReplace,
                        unsigned int objectIndex);
  void replaceParameter(EffectExpression &exp, unsigned int paramToReplace,
                        unsigned int objectIndex);
  void replaceParameter(Effect &eff, unsigned int paramToReplace,
                        unsigned int objectIndex);
  void replaceParameter(DurativeEffect &eff, unsigned int paramToReplace,
                        unsigned int objectIndex);
  void replaceParameter(TimedEffect &eff, unsigned int paramToReplace,
                        unsigned int objectIndex);
  void replaceParameter(AssignmentContinuousEffect &eff,
                        unsigned int paramToReplace, unsigned int objectIndex);
  void removeImplications(Precondition &prec);
  void removeImplications(DurativeCondition &prec);
  void removeImplications(Effect &eff);
  void removeImplications(DurativeEffect &eff);
  void removeImplications(GoalDescription &goal);
  void preconditionOptimization(Precondition *prec, Precondition *parent,
                                unsigned int termNumber, Action &a);
  void preconditionOptimization(DurativeCondition *prec,
                                DurativeCondition *parent,
                                unsigned int termNumber, DurativeAction &a);
  void preconditionOptimization(DurativeCondition *prec,
                                DurativeCondition *parent,
                                unsigned int termNumber,
                                DurativeEffect *parentEff);
  void negationOptimization(Precondition *prec, Precondition *parent,
                            unsigned int termNumber, Action &a);
  Comparator negateComparator(Comparator c);
  void effectOptimization(Effect *eff, Effect *parent, unsigned int termNumber,
                          Action &a);
  void effectOptimization(DurativeEffect *eff, DurativeEffect *parent,
                          unsigned int termNumber, DurativeAction &a);
  void effectOptimization(TimedEffect *eff, TimedEffect *parent,
                          unsigned int termNumber, DurativeEffect *parentEff);
  void negationOptimization(Effect *eff, Effect *parent,
                            unsigned int termNumber, Action &a);
  void negationOptimization(TimedEffect *eff, TimedEffect *parent,
                            unsigned int termNumber, DurativeEffect *parentEff);
  void goalOptimization(GoalDescription *goal, Precondition *parentPrec,
                        Effect *parentEff, GoalDescription *parent,
                        unsigned int termNumber);
  void goalOptimization(GoalDescription *goal, DurativeCondition *parentPrec,
                        GoalDescription *parent, unsigned int termNumber);
  void negationOptimization(GoalDescription *goal, Precondition *parentPrec,
                            Effect *parentEff, GoalDescription *parent,
                            unsigned int termNumber);
  void negationOptimization(GoalDescription *goal,
                            DurativeCondition *parentPrec,
                            GoalDescription *parent, unsigned int termNumber);
  bool existingConditionalEffects(Effect &eff);
  bool existingConditionalEffects(DurativeEffect &eff);
  void buildOperators(Action &a, bool isTIL, bool isGoal);
  void buildOperators(DurativeAction &a, bool isTIL, bool isGoal);
  std::vector<Operator> buildOperatorPrecondition(Precondition &prec, Action &a,
                                                  Operator *op);
  std::vector<Operator> buildOperatorPrecondition(DurativeCondition &prec,
                                                  DurativeAction &a,
                                                  Operator *op);
  std::vector<Operator> buildOperatorPrecondition(GoalDescription &goal,
                                                  Action &a, Operator *op);
  std::vector<Operator> buildOperatorPrecondition(GoalDescription &goal,
                                                  DurativeAction &a,
                                                  Operator *op);
  std::vector<Operator> buildOperatorPreconditionAnd(Precondition &prec,
                                                     Action &a, Operator *op,
                                                     unsigned int numTerm);
  std::vector<Operator> buildOperatorPreconditionAnd(DurativeCondition &prec,
                                                     DurativeAction &a,
                                                     Operator *op,
                                                     unsigned int numTerm);
  std::vector<Operator> buildOperatorPreconditionAnd(GoalDescription &goal,
                                                     Action &a, Operator *op,
                                                     unsigned int numTerm);
  std::vector<Operator> buildOperatorPreconditionAnd(GoalDescription &goal,
                                                     DurativeAction &a,
                                                     Operator *op,
                                                     unsigned int numTerm);
  bool checkValidOperator(Operator &op, unsigned int numParameters);
  bool setParameterValues(unsigned int paramValues[],
                          unsigned int equivalences[],
                          const std::vector<OpEquality> &equality);
  bool checkEqualities(unsigned int paramValues[], unsigned int equivalences[],
                       std::vector<OpEquality> &equality,
                       unsigned int numParameters);
  bool checkPreconditions(unsigned int paramValues[],
                          std::vector<OpFluent> &precs);
  void terminateBuildingOperator(Operator &op, Action &a, std::string name);
  void terminateBuildingOperator(Operator &op, DurativeAction &a,
                                 std::string name);
  void buildOperatorEffect(Operator &op, Effect &effect);
  void buildOperatorEffect(Operator &op, DurativeEffect &effect);
  void buildOperatorEffect(Operator &op, TimedEffect &effect,
                           TimeSpecifier time);
  void buildOperatorEffect(Operator &op, AssignmentContinuousEffect &effect,
                           TimeSpecifier time);
  void buildOperatorEffect(Operator &op, FluentAssignment &effect,
                           TimeSpecifier time);
  void buildOperatorEffect(Operator &op, GoalDescription &condition,
                           Effect &effect);
  void buildOperatorEffect(Operator &op, DurativeCondition &condition,
                           TimedEffect &effect);
  void buildConditionalEffectCondition(Operator *op, OpConditionalEffect *cond,
                                       GoalDescription *condition);
  void buildConditionalEffectCondition(Operator *op, OpConditionalEffect *cond,
                                       DurativeCondition *condition);
  void buildConditionalEffectEffect(Operator *op, OpConditionalEffect *cond,
                                    Effect *effect);
  void buildConditionalEffectEffect(Operator *op, OpConditionalEffect *cond,
                                    TimedEffect *effect, TimeSpecifier time);
  std::string getOpName(std::string &name, int index, int size);
  void conjuctionOptimization(Effect *eff);
  void conjuctionOptimization(DurativeEffect *eff);

public:
  Preprocess();
  ~Preprocess();
  PreprocessedTask *preprocessTask(ParsedTask *parsedTask);
};

#endif

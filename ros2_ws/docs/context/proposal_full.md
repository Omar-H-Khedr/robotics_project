# Exposé Template

## according to §5 of the Doctoral Regulations of the Doctoral Center 

## Engineering Sciences and Information Technologies (IWIT)

1.  Executive Summary

Smart manufacturing cells increasingly require robotic assembly skills that remain reliable under product changes, tolerance variation, and uncertain physical contact. In peg-in-hole assembly, small pose errors and tolerance stacks can shift the interaction from nominal insertion to edge contact, jamming, wedging, or recovery. Purely geometric planning is therefore insufficient once contact begins; robust execution requires closed-loop sensing, compliant interaction, and systematic validation under realistic disturbances (Lasi et al., 2014; Koren et al., 1999; Schumacher et al., 2019; Raibert & Craig, 1981; Whitney, 1982).

Recent research has advanced learning-based peg-in-hole assembly through virtual guiding-force reinforcement learning, efficient RL formulations, representation learning, meta-policy learning, multi-peg insertion control, and visual-tactile SAC-based learning under uncertainty (Zang et al., 2025; Gai et al., 2024; Zang et al., 2023; Yan et al., 2025; Li, D. et al., 2025a; Tang et al., 2025). However, an integrated framework that jointly addresses multi-modal contact-state disambiguation, fast context-based online adaptation across geometry and tolerance regimes, and safety-constrained sim-to-real deployment remains underdeveloped.

This doctoral project proposes a visuomotor, context-based meta-reinforcement learning framework for adaptable peg-in-hole assembly in smart manufacturing. The method combines RGB-D perception, force/torque sensing, and proprioceptive feedback to infer contact-relevant latent context from short interaction histories, drawing on recent advances in off-policy reinforcement learning and context-based meta-RL (Haarnoja et al., 2018; Rakelly et al., 2019). A context-conditioned SAC policy adapts online to geometry and tolerance variation, while a dual-layer safety architecture separates contact-phase compliance from pre-contact constraint enforcement. During insertion, an admittance-based virtual-force layer regulates physical interaction; during free-motion and approach phases, a runtime safety filter supervises hard operational constraints under specified collaborative-robot assumptions (Raibert & Craig, 1981; ISO/TS 15066, 2016).

The scientific contribution is organized around three connected pillars: multi-modal contact-state inference with latent context adaptation, safety-mediated execution through contact compliance and runtime constraint filtering, and simulation-backed validation using calibrated simulation, structured domain randomization, and explicit transfer gates. Safe Success is defined as task completion with zero hard constraint violations and is evaluated together with cycle time, peak wrench, safety-filter interventions, and adaptation speed under controlled tolerance stress tests. The simulation-backed validation strategy follows the broader role of calibrated digital models in smart manufacturing and digital-twin-based validation workflows (Kritzinger et al., 2018; Zhong et al., 2017).

The target platform is a KUKA LBR iisy 6 R1300 collaborative robot equipped with an Intel RealSense D405 RGB-D camera and force/proprioceptive feedback. The hardware serves as an industrially relevant validation platform; the primary contribution is the transferable methodology rather than the robot setup itself. The expected outcome is empirical evidence of safety, robustness, and adaptability within specified operational assumptions and bounded deployment conditions, supported by simulation-backed training, transfer gates, and real-robot benchmarking.

2.  Suggested Topic

The suggested topic of the doctoral project is:

**"Visuomotor Context-Based Meta-Reinforcement Learning with Virtual-Force Safety for Adaptable Peg-in-Hole Assembly in Smart Manufacturing"**

The topic targets adaptive industrial assembly in collaborative smart manufacturing cells, focusing on reliable peg-in-hole insertion under realistic part and process variability. Building on the motivation in the Executive Summary, the proposed method combines RGB-D perception with context-based reinforcement learning for closed-loop correction and rapid adaptation, while addressing operational safety through admittance/virtual-force mediation and a deployment-time runtime safety filter (see Section 5). The typical setup and the interaction between visual and haptic feedback in this process are illustrated in Figure 1.

| <img src="docs/context/proposal_media/media/image1.jpeg" style="width:2.59023in;height:2.37864in" /> | <img src="docs/context/proposal_media/media/image2.jpeg" style="width:2.92492in;height:2.09943in" /> |
|------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------|
| \(a\)                                                                                                | \(b\)                                                                                                |

**Figure 1:** Typical Robotic Peg-in-Hole Assembly Platform. (a) Physical experimental setup comprising a collaborative robot in a contact-rich insertion scenario, reproduced from Li et al. (2022). In this project, the real platform is the KUKA LBR iisy 6 R1300 with an external RGB-D sensor. (b) Corresponding physics-based simulation environment used for training reinforcement learning agents under controlled variability, reproduced from Tang et al. (2025). The combination of real-world sensing and physics-based simulation forms the basis for sim-to-real transfer workflows.

3.  State of Research

Given the motivation summarized in the Executive Summary, the focus here is on recent learning-based assembly methods and the remaining gap toward an integrated, safe, and adaptable peg-in-hole pipeline.

Peg-in-hole insertion is a widely used benchmark for contact-rich industrial assembly, where uncertainty in tolerance stacks and contact-mode transitions makes the true interaction state only partially observable once contact begins. In real cells, failure modes such as jamming and wedging arise from small clearances and pose errors, motivating closed-loop strategies that combine sensing with compliant control. Classical compliant motion approaches (e.g., impedance control and hybrid force–position control) provide principled tools for safe interaction, but typically require careful tuning and do not automatically generalize across changing geometries and contact <span dir="rtl"></span>conditions (Raibert & Craig, 1981; Whitney, 1982).

Recent work shows an accelerating shift from hand-crafted contact-state logic to learning-based control. Zang et al. (2025) propose a deep reinforcement learning approach that uses a feedforward “virtual guiding force” as the action representation, emphasizing safety considerations during insertion and drawing inspiration from human dragging-like behaviors. Addressing learning efficiency, Gai et al. (2024) highlight that standard RL formulations can be inefficient for peg-in-hole because global policy parameterization expands the explored state-action space; they propose a “local connection” RL approach to improve learning efficiency. Complementary to pure RL, Li, D. et al. (2025a) address the practical reality of multiple peg-in-hole operations, proposing deviation estimation and fast insertion control to handle visually inaccessible assembly phases and large combinatorial complexity. These studies collectively underline that modern peg-in-hole solutions must be robust, sample-efficient, and safe, and they must cope with partial observability during contact. Tang et al. (2025) is especially close to the proposed project because it combines visual-tactile fusion with SAC-based learning for peg-in-hole assembly under uncertain pose, geometry, force, and sensor-noise conditions. Its multimodal formulation is directly relevant to this thesis; however, it does not explicitly target latent context adaptation, contact-state disambiguation as a core modeling objective, or a dual-layer safety-constrained sim-to-real transfer protocol.

A key challenge is the gap between successful single-task training and robust generalization across variations. Representation choices strongly influence learnability: Zang et al. (2023) propose geometric-feature representations and pre-training for peg-in-hole RL to avoid redundant state dimensions and accelerate training stability. Beyond representation, generalization across categories motivates meta-learning: Yan et al. (2025) propose adaptive meta-policy learning with a virtual model set for multi-category peg-in-hole assembly, explicitly addressing low learning efficiency and low adaptability when moving beyond a single category. More broadly, the RL literature provides foundations for robust off-policy learning in continuous control via entropy-regularized actor-critic methods such as Soft Actor-Critic (SAC) (Haarnoja et al., 2018), and for context-based meta-RL via probabilistic context variables that infer task information from recent experience (Rakelly et al., 2019). These developments offer a principled route toward fast adaptation.

At the same time, recent robot-learning research has advanced substantially in two adjacent directions that are directly relevant to this project. First, modern visuomotor policies, including Diffusion Policy, have demonstrated strong performance in contact-rich tasks by providing scalable, expressive policy representations (e.g., Chi et al., 202<span dir="rtl">5</span>). Second, safe learning has seen rapid progress in runtime shielding and formally specified safety mechanisms—such as certified controllers and control-barrier-function-based filters—that enforce hard constraints under modeled conditions (Hsu et al., 2024; Knoedler et al., 2025; Römer et al., 2025). Although these advancements are extremely relevant, they typically address either general manipulation tasks without explicit assembly-specific contact disambiguation or safety filtering as a general low-level protection without integrating it into a context-adaptive peg-in-hole pipeline.

In addition to control techniques, flexible automation, fast changeovers, and system integration in production setups are highlighted by the broader development in smart manufacturing (Koren et al., 1999; Zhong et al., 2017). To support reproducible evaluation and safety-constrained sim-to-real transfer, this thesis relies on a calibrated physics-based simulation model (Kritzinger et al., 2018). Following this paradigm, this thesis utilizes a simulation-backed development process, with the detailed workflow and decision gates defined in Section 5.

Although recent work advances key components of learning-based assembly (representation learning, meta-policy adaptation, and safety-oriented execution), the literature still lacks a unified, reproducible pipeline tailored for these challenges. In particular, existing approaches do not yet combine RGB-D visuomotor perception, context-based meta-RL, an admittance/virtual-force contact mediator, a runtime safety filter for pre-contact constraints, and a simulation-backed sim-to-real workflow within a single reproducible framework. This motivates the integrated approach and evaluation protocol developed in this thesis (see Sections 4–5).

4.  Goals and Contribution of the Work

The aim of this doctoral project is to develop and validate an adaptive robotic peg-in-hole assembly capability that remains reliable under realistic variability and is deployable in collaborative smart manufacturing cells.

The central research question is: **How can a collaborative robot learn a safe, robust, and rapidly adaptable peg-in-hole insertion skill from RGB-D and force/proprioceptive feedback, such that performance generalizes across multiple part variants with minimal manual re-tuning?**

Crucially, the proposed pipeline does not replace standard geometric and kinematic robotics components; rather, it builds on them. Inverse kinematics, frame calibration, and pose estimation remain essential for pre-contact positioning, workspace feasibility, and nominal approach trajectory generation. In this hybrid architecture, classical geometric planning provides the nominal approach and initialization, while the learning-based closed-loop controller operates in the regime where geometric uncertainty and contact dynamics dominate, namely during contact onset, alignment correction, insertion, and recovery from disturbances. In this sense, the learning component complements rather than replaces classical robotics by addressing the regime in which partial observability, and contact-mode transitions limit purely geometric execution.

The research questions address three coupled aspects of the proposed methodology: multi-modal contact-state inference and adaptation, safety-mediated execution, and simulation-backed validation with transfer gates.

First, how should RGB-D perception and force/proprioceptive signals be encoded to form a compact yet sufficient state for closed-loop insertion under partial observability, and how does this choice compare to existing representation approaches (Zang et al., 2023)?

Hypothesis H1.1: A unified multi-modal representation (RGB-D + force/torque + proprioception) will achieve higher Safe Success and lower peak contact wrench than single-modality or dual-modality variants under controlled geometric and tolerance <span dir="rtl"></span>disturbances.

Hypothesis H1.2: Explicit contact-state disambiguation (e.g., free-space, initial touch, edge contact, jam, alignment/sliding, insertion) will improve adaptation stability and reduce failure modes, especially jamming and wedging, compared to feature representations that do not model contact-state ambiguity explicitly.

Second, can a context-based meta-RL formulation infer an informative latent context from short interaction histories in a way that improves adaptation to new peg/hole variants relative to single-task RL baselines (Haarnoja et al., 2018), and how does it compare to existing multi-category meta-policy approaches (Yan et al., 2025)?

Hypothesis H2.1: The context-based meta-RL formulation will reduce the number of adaptation episodes required to reach a target success rate on unseen variants compared to single-task SAC baselines trained separately per category.

Hypothesis H2.2: Context inference from short interaction buffers will improve cross-variant robustness, measured through Safe Success, cycle time, and recovery behavior, across continuous geometry and tolerance changes relative to category-level meta-policy baselines without explicit contact-state disambiguation.

Third, what is the measurable safety and robustness impact of mediating learned actions through an admittance/virtual-force layer during contact, and what additional protection is achieved by a deployment-time runtime safety filter during the pre-contact approach phase?

Hypothesis H3.1: An admittance/virtual-force mediation layer will reduce peak wrench and safety-violation frequency during contact while preserving or improving insertion success under disturbance conditions, compared to direct policy action execution.

Hypothesis H3.2: A deployment-time runtime safety filter enforcing hard kinematic constraints during the free-motion approach phase is expected to reduce pre-contact safety violations and prevent them within the specified operational constraints.

The intended scientific contributions are threefold. First, the project will deliver a visuomotor learning architecture that combines RGB-D perception, force/proprioceptive feedback, explicit contact-state disambiguation, and latent context inference to support rapid adaptation across peg/hole variants and physical regimes. Second, it will provide a safety-integrated control design in which contact-phase interaction is regulated through admittance-based virtual-force mediation, while pre-contact motion is governed by an independent runtime constraint-enforcement layer. Third, it will establish a reproducible evaluation and sim-to-real transfer protocol based on a calibrated physics-based simulation model, structured domain randomization, formal transfer gates, and stress-test benchmarking aligned with smart manufacturing requirements.

To highlight the novelty of the proposed framework, Table 1 positions it relative to representative recent assembly baselines as well as adjacent visuomotor policy-learning approaches relevant to contact-rich manipulation.

Table 1. Comparison of the proposed method with representative recent approaches

| **Aspect**             | **Tang et al. (2025) / Visual-tactile SAC**                                       | **Zang et al. (2025) / Virtual guiding force RL**           | **Yan et al. (2025) / Adaptive meta-policy** | **Chi et al.** <span dir="rtl">)</span>**2025**<span dir="rtl">(</span> **/ Diffusion Policy** | **Proposed method**                                                                       |
|------------------------|-----------------------------------------------------------------------------------|-------------------------------------------------------------|----------------------------------------------|------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------|
| **Perception / state** | RGB, depth, tactile force, robot pose via fusion network                          | Primarily force/proprioceptive insertion state              | Vision/category-oriented representation      | Strong visuomotor observation-based policy learning                                            | RGB-D + force/torque + proprioception with explicit contact-state modeling                |
| **Adaptation**         | Generalization to uncertainty and peg-shape variation; no explicit meta-context   | Task-specific robust RL for insertion                       | Meta-policy transfer across categories       | General policy expressiveness, not rapid context adaptation                                    | Latent context from short history for online adaptation across tolerance regimes          |
| **Safety**             | Improved robustness through learned SAC policy; no decoupled runtime safety layer | Safety emphasis through virtual guiding-force action design | Safety mostly implicit in training           | Safety requires external shielding/filtering                                                   | Admittance/virtual-force contact layer + runtime filter for pre-contact constraints       |
| **Validation**         | Simulation/experiments under uncertainty                                          | Sim-to-real insertion validation                            | Category generalization validation           | Broad manipulation tasks                                                                       | Safety-conditioned transfer gates, stress tests, Safe Success, peak wrench, interventions |

Finally, the ultimate deliverable is a transferable methodology rather than a single task-specific controller. The project defines and evaluates how multi-modal state representations and latent context inference are constructed, how context-adaptive policies are trained and safety-constrained before deployment, and how generalization across variants is benchmarked in a reproducible manner. Beyond peg-in-hole assembly, the expected scientific outcome is an evidence-based methodological framework for contact-rich industrial skills with similar structure, including connector mating, press-fit operations, and other tolerance-sensitive insertion tasks, thereby reducing manual tuning effort and accelerating deployment across applications.

5.  Description of Procedures and Planned Methods

The proposed methodology combines multi-modal perception, context-based adaptation, and safety-mediated execution within a calibrated physics-based simulation workflow. Figure 2 summarizes the information flow from sensing to context inference, policy output, virtual-force mediation, and compliant execution.

**5.1 Perception and Context Inference**

RGB-D observations provide relative pose, local geometry cues, and approach alignment, while force/torque and proprioceptive signals provide information about contact onset, resistance, and motion state. Because the contact mode is not fully observable from a single instant, the system maintains a short-horizon context buffer over the last K control steps, typically K = 25-75 at 50 Hz. This buffer contains encoded visual features, wrench signals, proprioception, previous actions, and event flags.

The context module maps this history to a latent variable z representing the task variant and current contact situation. To keep the doctoral scope focused, the main implementation will use a deterministic sequence encoder, such as a GRU or compact Transformer. PEARL-style probabilistic context variables inform the design conceptually, but the deterministic-versus-variational encoder choice is treated as an implementation design decision rather than a separate research question. The evaluated comparisons will focus on no-context, fixed-context, and history-based context variants.

**5.2 Learning-Based Policy**

The control policy is implemented as a context-conditioned stochastic actor-critic, using Soft Actor-Critic (SAC) as the learning backbone because of its off-policy sample efficiency and robustness in continuous-control settings (Haarnoja et al., 2018). The policy receives the fused state representation together with the inferred latent context ***z*** and outputs continuous motion or force-guidance commands appropriate for the current phase of insertion. Training begins in simulation, where large-scale experience can be collected under controlled variability. Domain randomization is applied over factors such as part tolerances, camera noise, initial misalignment, contact <span dir="rtl"></span>conditions, and compliance parameters in order to improve robustness and reduce sim-to-real mismatch. Representation-level refinements, including optional pre-training, may be incorporated where useful to reduce redundancy in the learned state description and improve training stability, consistent with prior work on peg-in-hole representation learning (Zang et al., 2023). To quantify the effect of the main architectural choices, the study includes targeted ablations focused on the key claims of the thesis: (A1) removal of context inference to evaluate the meta-adaptation mechanism, (A2) modality dropout to assess sensitivity to partial observability, and (A3) safety ablations separating removal of the admittance/virtual-force mediator from removal of the runtime safety filter. These ablations are evaluated primarily through Safe Success, adaptation speed, peak wrench, and intervention behavior under controlled disturbances.

**5.3 Safety-Mediated Execution**

To deploy learned actions on a collaborative robot, the policy output is not executed in raw form. Instead, execution is mediated through a dual-layer safety architecture that separates contact regulation from pre-contact constraint enforcement. During physical interaction, an admittance-based virtual-force layer modulates compliance, bounds interaction forces, and shapes motion in a way that preserves task progress while reducing the likelihood of unsafe contact events. This layer therefore acts as the primary mechanism for safe contact-phase behavior and is directly aligned with the collaborative setting of the target platform (Zang et al., 2025; ISO/TS 15066, 2016).

In the free-motion and pre-contact approach phases, a deployment-time runtime safety filter supervises velocity limits, workspace boundaries, and approach-region constraints. At the methodological level, this filter may project, override, or stop unsafe commands, thereby separating pre-contact constraint enforcement from contact-phase compliance.

**5.<span dir="rtl">4</span> Simulation-Backed Workflow and Transfer Gates**

The project uses a calibrated physics-based simulation model of the KUKA LBR iisy assembly cell for training, ablation, stress testing, and transfer preparation. Policies are first evaluated in simulation and then advanced to conservative real-robot deployment only after passing predefined transfer gates. Example gates include at least 90% Safe Success over the defined simulation stress tests, zero hard pre-contact constraint violations, and bounded contact wrench below predefined limits aligned with the collaborative deployment assumptions.

**5.<span dir="rtl">5</span> Evaluation Protocol**

The evaluation design follows recent benchmarking practice for robotic assembly (Li, S. et al., 2025b) and is organized around task performance, interaction safety, and adaptability. The primary metrics are Safe Success, adaptation speed on unseen variants, hard constraint violations per episode, and runtime safety-filter interventions per episode. Secondary metrics include nominal success rate, insertion time, peak contact force, cumulative wrench, and recovery frequency after misalignment. Adaptability is assessed using standard meta-learning perspectives (Yan et al., 2025; Rakelly et al., 2019), including both pre-adaptation performance and the number of episodes required to recover high performance on previously unseen peg/hole variants under controlled geometry and tolerance changes. Taken together, this evaluation protocol is intended to show not only whether the method succeeds, but under which physical conditions it remains safe, transferable, and practically useful.

<img src="docs/context/proposal_media/media/image3.png" style="width:6.3in;height:1.33681in" />

**Figure 2:** Proposed end-to-end framework for multi-modal perception, latent context inference, and safety-mediated execution on the collaborative peg-in-hole assembly platform.

<img src="docs/context/proposal_media/media/image4.png" style="width:6.3in;height:1.14958in" />

**Figure 3:** Research workflow with validation gates and rework loops, linking simulation training, safety verification, sim-to-real transfer, and real-world adaptation.

6.  Resources and Time Planning

**6.1 Technical Resources and Infrastructure**

Executing the proposed research requires (i) a collaborative robotic assembly cell capable of safe physical interaction, (ii) multi-modal sensing for closed-loop correction, and (iii) GPU-enabled compute and simulation infrastructure for training, ablation, and transfer evaluation within a calibrated physics-based simulation workflow. These requirements are met by the on-site experimental setup and software environment at the Robotics and Additive Manufacturing Lab at Galala University, which provides the physical and computational basis for the planned research program.

**Robotic hardware.** The core experimental platform consists of a KUKA LBR iisy 6 R1300 collaborative robot. This platform is particularly suitable for contact-rich assembly because its integrated joint-torque sensing supports safe physical interaction and is directly relevant to the planned admittance-based contact mediation during insertion. Visual sensing is provided by an Intel RealSense D405 RGB-D camera, selected for reliable close-range depth perception during peg/hole pre-alignment, local correction, and observation of contact-relevant geometric cues.

**Computational resources.** Training and simulation require GPU-enabled compute for large-scale rollout generation and iterative model development. The project has access to high-performance workstations equipped with NVIDIA RTX GPUs, enabling efficient training of context-based and meta-RL agents as well as repeated stress-test evaluation under structured variability.

**Software stack.** The software framework relies on ROS 2 for hardware abstraction, experiment orchestration, real-time communication, and logging, and on PyTorch for implementation of the perception encoder, context inference module, and reinforcement-learning backbone. The simulation environment will be developed in NVIDIA Isaac Sim, which supports contact-rich interaction, structured parameter variation, and sim-to-real workflows. This environment allows systematic control over tolerances, contact properties, sensor noise, and initial alignment conditions, thereby supporting both training and transfer preparation in a reproducible setting.

<img src="docs/context/proposal_media/media/image5.jpeg" style="width:4.20213in;height:3.02885in" />

**Figure 4:** Experimental infrastructure at the Robotics and Additive Manufacturing Lab at Galala University (original photograph by the author), including the KUKA LBR iisy collaborative robot, a GPU-enabled workstation for RL training, and supporting equipment for simulation-model calibration and rapid prototyping of assembly variants.

Taken together, this infrastructure ensures that the project can be executed primarily within the candidate’s day-to-day research environment while maintaining continuity between simulation development, algorithmic iteration, and physical validation on the target robotic platform.

**6.2 Candidate Qualifications and Preliminary Work**

The candidate has the methodological and technical background required to execute the proposed project, as demonstrated by prior work in deep learning, multi-modal perception, control-oriented systems, and engineering education.

**Teaching and applied supervision.** The candidate has extensive academic experience as an Assistant Lecturer at Galala University, teaching specialized courses including Robotics, Modeling and Simulation of Mechatronics Systems, and Advanced Manufacturing. In this role, he has supervised graduation and industrial projects involving robotic implementation, simulation-based analysis, and practical engineering integration, including work connected to industrial collaborators such as Ezz Steel.

**Deep learning and physics-informed AI.** The candidate has developed advanced neural architectures, including Physics-Informed Kolmogorov–Arnold Networks (PI-KANs), currently under review in Theoretical and Computational Fluid Dynamics. This work demonstrates experience in custom model development, quantitative evaluation, and technically rigorous experimentation, all of which are directly relevant to the proposed learning framework.

**Computer vision and sensor fusion.** The candidate has prior experience in multi-modal perception, including work on RGB–thermal sensor fusion for safety-critical detection tasks. This background provides a solid methodological foundation for the proposed combination of RGB-D, force/torque, and proprioceptive feedback in robotic assembly.

**Robotics and control.** The candidate holds a Master’s degree in Mechatronics with a focus on electro-hydraulic positioning systems, providing a strong background in control, system modeling, and physically grounded motion design relevant to compliant and safety-aware robotic manipulation.

Overall, the candidate’s prior work establishes a suitable foundation for carrying out the project independently on a day-to-day basis while benefiting from structured scientific supervision and collaborative feedback during the doctoral period.

**6.3 Work Schedule, Milestones, and Supervision Structure**

The work plan is organized over 36 months, as outlined in Figure 5. The schedule is intentionally designed with partial overlaps between simulation development, algorithmic refinement, transfer preparation, and real-robot validation so that the project progresses as an iterative research program rather than as a rigid sequential waterfall.

In the early phase, the focus is on consolidating the state of research, integrating the experimental setup, establishing reproducible baselines, and building the initial calibrated physics-based simulation model. Simulation-based training begins while the model is still being refined, allowing calibration, representation design, and baseline evaluation to evolve together. Safety-layer integration and sim-to-real calibration are initiated before all simulation work is complete, which supports progressive validation and avoids compressing transfer risks into the final stage of the project. Benchmarking, manuscript preparation, and thesis writing begin before all experiments conclude, ensuring that dissemination follows naturally from matured work packages rather than being postponed to the end.

To manage technical risk, the work plan uses explicit decision gates. These gates determine whether the simulation model is sufficiently calibrated, whether simulation policies meet robustness and safety thresholds, whether transfer to the physical robot remains acceptable under conservative limits, and whether online refinement can proceed without unsafe events. For example, progression from simulation to first real-robot deployment requires at least 90% Safe Success in simulation across the defined tolerance stress tests, together with zero hard constraint violations and bounded contact force/wrench below predefined safety limits aligned with ISO/TS 15066. Advancing from conservative deployment to active online fine-tuning further requires stable Safe Success over a predefined real-trial window, while safety-filter interventions are logged and analyzed separately.

| **Months** | **Phase**                   | **Main focus**                                                              | **Deliverables**                                            |
|------------|-----------------------------|-----------------------------------------------------------------------------|-------------------------------------------------------------|
| **1-6**    | Foundations                 | Literature, cell integration, baseline compliant control, logging           | Baseline controllers and logging pipeline                   |
| **4-12**   | Simulation + representation | Physics simulation, calibration, domain randomization, representation tests | Calibrated simulation v1 and simulation benchmarks          |
| **10-22**  | Core learning               | SAC baselines, context-based meta-RL, ablations                             | Trained policies and adaptation analysis                    |
| **18-33**  | Transfer + safety           | Safety layers, sim-to-real calibration, conservative deployment             | Safety-constrained transfer protocol and real-robot results |
| **24-36**  | Benchmark + thesis          | Stress testing, dissemination, thesis writing                               | Final benchmarks, manuscripts, thesis submission            |

**Risk analysis with decision gates.** The dominant risks are sim-to-real mismatch, insufficient adaptation across variants, and unsafe contact events during real testing. These risks are mitigated through explicit variability modeling in simulation, a staged deployment protocol, conservative safety thresholds, and mandatory decision gates before each escalation step. A further mitigation mechanism is the runtime safety filter, which acts as a deployment gatekeeper during real experiments. Sample inefficiency is addressed through off-policy learning, structured simulation experience, and context-based adaptation, while rework loops are built into the plan so that observed failures can be used to update noise assumptions, calibration parameters, and training distributions.

**Supervision and coordination structure.** The experimental work will be conducted primarily at Galala University using the available laboratory infrastructure described above. Scientific supervision will be maintained through regular online meetings, milestone-based progress reviews, and iterative manuscript feedback with the supervisors. This structure is intended to ensure continuity of guidance despite the geographical distance, while allowing the candidate to carry out day-to-day implementation, experimentation, and data collection within the local research environment. Where useful and feasible, focused in-person coordination meetings can complement the regular remote supervision at key project milestones.

**Mobility and defense commitment.** The candidate confirms his full willingness to travel to Germany for the final dissertation defense at Hochschule Harz. Travel, funding, and visa arrangements will be planned in due time to ensure physical presence for this mandatory milestone and, where required, for selected coordination meetings during the doctoral period.

**Institutional cooperation (MoU).** Beyond the scientific goals of the thesis, the candidate and the supervisors intend to explore a Memorandum of Understanding (MoU) between Galala University and Harz University of Applied Sciences during the doctoral period. The aim is to create a framework for sustained academic cooperation, including future student exchange and joint research activities beyond the scope of the present project.

<img src="docs/context/proposal_media/media/image6.png" style="width:11.22989in;height:2.11603in" />

**Figure 5.** 36-month Gantt chart with overlaps and decision gates (DG), showing parallel work streams, explicit dependencies, and publication milestones scheduled after completion of the corresponding work packages.

7.  Intended Publications and Proposed Peer-Reviewed Journals

The results are intended for dissemination through leading peer-reviewed robotics conferences and suitable Q1 journals. The plan follows the thesis structure:

Paper 1, Months 12-18**: “A Simulation-Backed Framework for Safe Sim-to-Real Reinforcement Learning in Contact-Rich Assembly”,** focusing on simulation, domain randomization, and baseline benchmarking.

Paper 2, Months 20-26: **“Safe Context-Based Meta-Reinforcement Learning for Peg-in-Hole Assembly with Contact-State Inference and Dual-Layer Safety Mediation”,** focusing on the core methodological contribution.

Paper 3, Months 30-36: **“Robust Sim-to-Real Transfer for High-Mix Assembly: Real-Robot Benchmarking under Safety Constraints”,** focusing on real-robot validation and safety-conditioned transfer.

The first two contributions are planned for submission to robotics venues such as ICRA, IROS, or CASE, including suitable IEEE Robotics and Automation Letters conference-option routes where appropriate. Consolidated methodological and experimental findings will then be prepared for a suitable Q1 journal aligned with the project focus, such as IEEE Robotics and Automation Letters, Engineering Applications of Artificial Intelligence, Robotics and Computer-Integrated Manufacturing, Advanced Engineering Informatics, or Robotics and Autonomous Systems. Final venue selection will depend on scientific fit, maturity of results, feasibility within the doctoral timeline, and suitable institutional open-access routes where applicable.

8.  References

Chi, C., Xu, Z., Feng, S., Cousineau, E., Du, Y., Burchfiel, B., Tedrake, R., and Song, S. (2025). Diffusion policy: Visuomotor policy learning via action diffusion. The International Journal of Robotics Research, 44(10-11), 1684-1704. https://doi.org/10.1177/02783649241273668

Gai, Y., Zhang, J., Wu, D., and Chen, K. (2024). Local connection reinforcement learning method for efficient robotic peg-in-hole assembly. Engineering Applications of Artificial Intelligence, 133, Article 108520. https://doi.org/10.1016/j.engappai.2024.108520

Haarnoja, T., Zhou, A., Abbeel, P., and Levine, S. (2018). Soft Actor-Critic: Off-policy maximum entropy deep reinforcement learning with a stochastic actor. Proceedings of the 35th International Conference on Machine Learning, PMLR 80, 1861-1870.

Hsu, K.-C., Hu, H., and Fisac, J. F. (2024). The safety filter: A unified view of safety-critical control in autonomous systems. Annual Review of Control, Robotics, and Autonomous Systems, 7, 47-72. https://doi.org/10.1146/annurev-control-071723-102940

ISO/TS 15066. (2016). Robots and robotic devices - Collaborative robots. International Organization for Standardization.

Knoedler, L., So, O., Yin, J., Black, M., Serlin, Z., Tsiotras, P., Alonso-Mora, J., and Fan, C. (2025). Safety on the fly: Constructing robust safety filters via policy control barrier functions at runtime. IEEE Robotics and Automation Letters, 10(10), 10058-10065. https://doi.org/10.1109/LRA.2025.3597847

Koren, Y., Heisel, U., Jovane, F., Moriwaki, T., Pritschow, G., Ulsoy, G., and Van Brussel, H. (1999). Reconfigurable manufacturing systems. CIRP Annals, 48(2), 527-540. https://doi.org/10.1016/S0007-8506(07)63232-6

Kritzinger, W., Karner, M., Traar, G., Henjes, J., and Sihn, W. (2018). Digital twin in manufacturing: A categorical literature review and classification. IFAC-PapersOnLine, 51(11), 1016-1022. https://doi.org/10.1016/j.ifacol.2018.08.474

Lasi, H., Fettke, P., Kemper, H.-G., Feld, T., and Hoffmann, M. (2014). Industry 4.0. Business & Information Systems Engineering, 6(4), 239-242. https://doi.org/10.1007/s12599-014-0334-4

Li, D., Li, X., Zhao, H., Ge, D., and Ding, H. (2025a). Robotic multiple peg-in-hole assembly with deviation estimation and fast insertion control. IEEE Transactions on Industrial Electronics, 72(1), 670-680. https://doi.org/10.1109/TIE.2024.3406860

Li, S., Gong, H., Liu, J., Li, J., and Deng, X. (2025b). Advances in robotic peg-in-hole assembly: A comprehensive review. Chinese Journal of Mechanical Engineering, 38, Article 196. https://doi.org/10.1186/s10033-025-01349-w

Li, S., Yuan, X., and Niu, J. (2022). Robotic peg-in-hole assembly strategy research based on reinforcement learning algorithm. Applied Sciences, 12(21), Article 11149. https://doi.org/10.3390/app122111149

Raibert, M. H., and Craig, J. J. (1981). Hybrid position/force control of manipulators. ASME Journal of Dynamic Systems, Measurement, and Control, 103(2), 126-133. https://doi.org/10.1115/1.3139652

Rakelly, K., Zhou, A., Quillen, D., Finn, C., and Levine, S. (2019). Efficient off-policy meta-reinforcement learning via probabilistic context variables. Proceedings of the 36th International Conference on Machine Learning, PMLR 97, 5331-5340.

Römer, R., Balletshofer, J., Thumm, J., Pavone, M., Schoellig, A. P., and Althoff, M. (2025). From demonstrations to safe deployment: Path-consistent safety filtering for diffusion policies. arXiv:2511.06385. https://doi.org/10.48550/arXiv.2511.06385

Schumacher, M., Wojtusch, J., Beckerle, P., and von Stryk, O. (2019). An introductory review of active compliant control. Robotics and Autonomous Systems, 119, 185-200. https://doi.org/10.1016/j.robot.2019.06.009

Tang, J., Yuan, X., and Li, S. (2025). Visual-tactile fusion and SAC-based learning for robot peg-in-hole assembly in uncertain environments. Machines, 13(7), Article 605. https://doi.org/10.3390/machines13070605

Whitney, D. E. (1982). Quasi-static assembly of compliantly supported rigid parts. ASME Journal of Dynamic Systems, Measurement, and Control, 104(1), 65-77. https://doi.org/10.1115/1.3149634

Yan, S., Tao, X., Ma, X., Hao, T., and Xu, D. (2025). Adaptive meta policy learning with virtual model for multi-category peg-in-hole assembly skills. IEEE Transactions on Industrial Informatics, 21(2), 1605-1614. https://doi.org/10.1109/TII.2024.3485768

Zang, Y., Wang, P., Zha, F., Guo, W., Ruan, S., and Sun, L. (2023). Geometric-feature representation based pre-training method for reinforcement learning of peg-in-hole tasks. IEEE Robotics and Automation Letters, 8(6), 3478-3485. https://doi.org/10.1109/LRA.2023.3261759

Zang, Y., Wang, Z., Pan, M., Hou, Z., Ding, Z., and Zhao, M. (2025). Safe peg-in-hole automatic assembly using virtual guiding force: A deep reinforcement learning solution. Robotics and Autonomous Systems, 185, Article 104894. https://doi.org/10.1016/j.robot.2024.104894

Zhong, R. Y., Xu, X., Klotz, E., and Newman, S. T. (2017). Intelligent manufacturing in the context of Industry 4.0: A review. Engineering, 3(5), 616-630. https://doi.org/10.1016/J.ENG.2017.05.015

The dissertation is planned to be written in the following language:

> ☐ German
>
> ☒ English

**Confirmation of the first supervisor**

I hereby confirm that the resource and schedule developed for the intended doctoral project is appropriate and that the necessary resources are available to carry out the project.

Date and signature of the first supervisor

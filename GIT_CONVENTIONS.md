<div class="phabricator-remarkup"><h2 class="remarkup-header"><a name="git-conventions"></a>Git conventions</h2>

<h3 class="remarkup-header"><a name="getting-started"></a>Getting started</h3>

<p>All commands shown here need to be executed in your terminal</p>

<p>For installing Git on Windows you can visit this website:&nbsp;<tt>https://git-scm.com/</tt></p>

<p>If you're on Linux then usually the 'git' package is available through your favorite package manager</p>

<p>You start by visiting the terminal:</p>

<p>It is required to set your email and username for your local Git configuration, example:</p>

<p><tt>git config --global user.email "i.ivanov.peev@hotmail.com"</tt></p>
<p><tt>git config --global user.name "IPeev1"</tt></p>

<p>Then you need to visit the remote repository</p>

<p>HHS-RP6: <tt class="remarkup-monospaced">https://github.com/IPeev1/HHS-RP6</tt></p>

<p>To create a local repository on you need to make a clone</p>

<p><tt class="remarkup-monospaced">git clone https://github.com/IPeev1/HHS-RP6.git</tt></p>

<p>Do this in the directory where you would want to store the project folder, i.e. repository</p>

<h3 class="remarkup-header"><a name="starting-a-new-feature"></a>Starting a new feature</h3>

<p>After cloning the repository, it’s time to create a feature branch on your local machine. Make sure you always branch of the develop branch.</p>

<p>Switch to the develop branch</p>

<p><tt class="remarkup-monospaced">git checkout develop</tt></p>

<p>Make sure the develop branch is in sync with the remote repository</p>

<p><tt class="remarkup-monospaced">git pull origin develop</tt></p>

<p>Fix merge issues if any arise</p>

<p>Branch off to a new feature branch</p>

<p>Stick to the branch naming convention. We use an adapted version of the wp-calypso naming convention, which looks as followed:</p>

<p><tt class="remarkup-monospaced">feature/{first_name_developer}/{add|update|fix|try}/{something}</tt></p>

<p>For instance if I was to work on a new feature, such as creating the possibility for serial communication, my branch name would be:</p>

<p><tt class="remarkup-monospaced">feature/ivan/add/serialCommunication</tt></p>

<p>_Note camelCasing where necessary_</p>

<p>There are two ways to create a new branch</p>

<p><tt>git branch feature/ivan/add/serialCommunication</tt></p>

<p>or</p>

<p><tt>git checkout -b feature/ivan/add/serialCommunication</tt></p>

<p>I myself prefer the latter but use the one you want</p>

<h3 class="remarkup-header"><a name="making-commits-to-your-b"></a>Commit structure</h3>

<p>Make commits often, push to your branch regular. Commits should follow a naming convention. The format looks like this:</p>

<p><tt class="remarkup-monospaced">ACTION: [AUDIENCE:] COMMIT_MSG [!TAG ...] [task|issue|bug]</tt></p>

<p><strong>Action</strong> is what the change is about, this can be new, chg, fix.</p>

<p>•	‘chg’: is for refractor, small improvements, cosmetic changes</p>

<p>•	‘fix’: is for bug fixes</p>

<p>•	‘new’: is for new features or big improvements</p>

<p><strong>Audience</strong> is optional and is WHO is concerned by the change:</p>

<p>•	‘dev’: Developers (API changes, refractors)</p>

<p>•	‘usr’: End-users (UI changes)</p>

<p>•	‘test’: Testers (Test only related changes)</p>

<p>•	‘doc’: Documentation changes</p>

<p><strong>Commit message</strong>: keep it short, clear and in English please</p>

<p><strong>Tags</strong> are additional adjectives giving extra clarification. They should preceded with a ! and common tags are:</p>

<p>•	‘refractor’</p>

<p>•	‘minor’ for insignificantly small changes (fix type, add a comment)</p>

<p>•	‘cosmetic’ for instance if linting forces you to update the code</p>

<p>•	‘wip’ for partial functionality but completed subfunctionality</p>

<p><tt class="remarkup-monospaced">Ref task T123</tt></p>

<p>If it fixes the bug or issue you can close it by adding:</p>

<p><tt class="remarkup-monospaced">Fixes task T123</tt> or <tt class="remarkup-monospaced">Closes bug T123</tt></p>

<p>Examples:</p>

<p><tt class="remarkup-monospaced">new: usr: support of bazaar implemented Ref task T123</tt></p>

<p><tt class="remarkup-monospaced">chg: re-indentend some lines !cosmetic Closes task T123</tt></p>

<p><tt class="remarkup-monospaced">new: dev: updated code to be compatible with last version of killer lib. Closes issue T123</tt></p>

<p><tt class="remarkup-monospaced">fix: pkg: updated year of licence coverage. Ref task T123</tt></p>

<p><tt class="remarkup-monospaced">new: test: added a bunch of test around user usability of feature X. Ref task T123</tt></p>

<p><tt class="remarkup-monospaced">fix: typo in spelling my name in comment. !minor Ref task T123</tt></p>

<h3>Making a commit</h3>

<p>To create a commit do the following:</p>

<p>Check first all the files in which you have made changes</p>

<p><tt>git status</tt></p>

<p>Add all the files to the staging area</p>

<p><tt>git add .</tt></p>

<p>The dot "." means all files in the current directory, you can also add each file individually</p>

<p><tt>git add file1.c</tt></p>
<p><tt>git add dir/file2.c</tt></p>
<p><tt>etc.</tt></p>

<p>But that is not very productive, afterwards execute</p>

<p><tt>git status</tt></p>

<p>to see if all the files are in the staging area, i.e. are highlighted green</p>

<p>Then you can commit all the files, this can be done in a few ways but the easiest one is</p>

<p><tt>git commit -m "new: dev: example commit message. Closes issue XXXX"</tt></p>

<h3 class="remarkup-header"><a name="finishing-a-feature"></a>Finishing a feature</h3>

<p>After all commits are made and your feature is finished the following actions should be done:</p>

<p>Check again to see if the code you've written works correctly, you can never be too sure if you forgot something</p>

<p>After you have assured that everything works and complies you can file a create request to the remote repository. First you need to push the feature branch to the remote repository</p>

<p><tt>git push origin feature/ivan/add/serialCommunication</tt></p>

<p>When you check the repository in GitHub you will see that your branch has been added. You’ll find a “new pull request” button at the top of your Github repository page. Make sure you select your feature branch as base and you compare it against the “develop” branch of the remote repository. If your branch is not mergeable you will need to rebase it with the remote “develop” branch. To do this you need to do the following:</p>

<p>Check all the branches with</p>

<p><tt>git branch</tt></p>

<p>You'll get a list of all branches in your local repository, and your own branch is highlighted. Let's say my branch is <tt>feature/ivan/add/serialCommunication</tt> and I have committed all the changes for the feature. What I need to do first is go to my local develop branch and update it to the most recent version of the remote develop branch</p>

<p><tt>git checkout develop</tt></p>
<p><tt>git pull origin develop</tt></p>

<p>Then switch back to your feature branch</p>

<p><tt>git checkout feature/ivan/add/serialCommunication</tt></p>

<p>Check to see if your commit is in the current branch with</p>

<p><tt>git log</tt></p>

<p>In order to update the feature branch with the current develop branch you will first need to remove the commit with</p>

<p><tt>git reset --soft HEAD~1</tt></p>

<p>Mind the <tt>--soft</tt> option, that means that the commit will be removed and your code will be returned to the staging area, if you add the <tt>--hard</tt> instead then the commit will be removed and all the code with it, and that isn't something you'd usually want, next you'll need to stash all your changes</p>

<p><tt>git stash</tt></p>

<p>What this does is it sets all the changes aside in a safe place, next you want to update the feature branch with the develop branch</p>

<p><tt>git rebase develop</tt></p>

<p>Then check your current stash</p>

<p><tt>git stash list</tt></p>

<p>You'll see a list of all your stashes, your most recent stash will be called <tt>stash@{0}</tt></p>

<p>There are a number of ways to set back the changes from the stash to the branch, to of which are:</p>

<p>First setting the changes back and then removing the stash from the list</p>

<p><tt>git stash apply stash@{0}</tt></p>
<p><tt>git stash drop stash@{0}</tt></p>

<p>or doing it in one command</p>

<p><tt>git stash pop stash@{0}</tt></p>

<p>The first one is the paranoid way, the second is the less paranoid way</p>

<p>After this is done your changes will be back in your branch and you can commit them again with the same message and push to the remote repository</p>

<p><tt>git push origin feature/ivan/add/serialCommunication -f</tt></p>

<p>The <tt>-f</tt> means that you will force the remote branch to accept the current one, otherwise you will get an error that the remote branch is up to date with the local one, which in this case is not correct</p>

<p>When this has all been done then you can continue creating the pull request in GitHub</p>

<h3 class="remarkup-header"><a name="last-step"></a>Last step</h3>

<p>After your pull request is accepted and merged to the central repository (wait for the merge as there might be adjustments needed on your side!) you can switch back to the develop branch:</p>

<p><tt class="remarkup-monospaced">git checkout develop</tt></p>

<p>Update the develop branch:</p>

<p><tt class="remarkup-monospaced">git pull origin develop</tt></p>

<p>And start branching off for your new feature.</p>

<h3 class="remarkup-header"><a name="references"></a>References</h3>

<p>Git cheat sheet: <a href="https://services.github.com/on-demand/downloads/github-git-cheat-sheet.pdf" class="remarkup-link" target="_blank" rel="noreferrer">https://services.github.com/on-demand/downloads/github-git-cheat-sheet.pdf</a></p>

<p>Git flow tutorial: <a href="https://www.atlassian.com/git/tutorials/comparing-workflows/forking-workflow" class="remarkup-link" target="_blank" rel="noreferrer">https://www.atlassian.com/git/tutorials/comparing-workflows/forking-workflow</a></p>

<p>Git branch naming: <a href="https://github.com/Automattic/wp-calypso/blob/master/docs/git-workflow.md" class="remarkup-link" target="_blank" rel="noreferrer">https://github.com/Automattic/wp-calypso/blob/master/docs/git-workflow.md</a></p>

<p>Git rebasing: <a href="https://github.com/edx/edx-platform/wiki/How-to-Rebase-a-Pull-Request" class="remarkup-link" target="_blank" rel="noreferrer">https://github.com/edx/edx-platform/wiki/How-to-Rebase-a-Pull-Request</a></p>

<p>Git commit name convention: <a href="https://github.com/vaab/gitchangelog/blob/master/gitchangelog.rc.reference" class="remarkup-link" target="_blank" rel="noreferrer">https://github.com/vaab/gitchangelog/blob/master/gitchangelog.rc.reference</a></p></div>
